
#include "coneskf_core/so3_math.hpp"
#include "coneskf_core/pose_tube.hpp"
#include <Eigen/Dense>
#include <deque>
#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>

// Inline updated ConformalBettingMartingale
class TunedMartingale {
public:
    TunedMartingale(
        size_t window_size = 200,
        size_t min_calibration_samples = 100,
        double epsilon_bet = 0.16,
        double alpha_far = 0.01,
        uint64_t seed = 42
    ) : window_size_(window_size),
        min_calibration_samples_(min_calibration_samples),
        epsilon_bet_(epsilon_bet),
        alpha_far_(alpha_far),
        threshold_(1.0 / alpha_far),
        wealth_(1.0),
        is_quarantined_(false),
        rng_(seed),
        unif_dist_(0.0, 1.0) {}

    double update(double s_k) {
        if (is_quarantined_) return wealth_;
        if (history_.size() < min_calibration_samples_) {
            history_.push_back(s_k);
            return wealth_;
        }

        size_t count_greater = 0, count_equal = 0;
        for (double s_past : history_) {
            if (s_past > s_k) count_greater++;
            else if (std::abs(s_past - s_k) < 1e-9) count_equal++;
        }

        double u_k = unif_dist_(rng_);
        double p_k = (static_cast<double>(count_greater) + u_k * (count_equal + 1.0)) / (history_.size() + 1.0);
        p_k = std::clamp(p_k, 1e-6, 1.0 - 1e-6);

        double e_k = 1.0 + epsilon_bet_ * (1.0 - 2.0 * p_k);
        wealth_ *= e_k;

        if (wealth_ < threshold_) {
            history_.push_back(s_k);
            if (history_.size() > window_size_) history_.pop_front();
        } else {
            is_quarantined_ = true;
        }

        return wealth_;
    }

    bool isQuarantined() const { return is_quarantined_; }
    double getWealth() const { return wealth_; }

private:
    size_t window_size_;
    size_t min_calibration_samples_;
    double epsilon_bet_;
    double alpha_far_;
    double threshold_;
    double wealth_;
    bool is_quarantined_;
    std::deque<double> history_;
    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> unif_dist_;
};

// Inline updated ConESKFFilter with TunedMartingale
struct State15 {
    Eigen::Vector3d p, v, ba, bg;
    Eigen::Matrix3d R;
    State15() { p.setZero(); v.setZero(); R.setIdentity(); ba.setZero(); bg.setZero(); }
};

struct ACIChan {
    double alpha_target = 0.05, alpha_current = 0.05, gamma_lr = 0.02;
    double current_quantile = 3.0, current_inflation = 1.0;
    std::deque<double> norm_history;
    size_t window_size = 200;

    double update(double s_k) {
        norm_history.push_back(s_k);
        if (norm_history.size() > window_size) norm_history.pop_front();

        std::vector<double> sorted(norm_history.begin(), norm_history.end());
        std::sort(sorted.begin(), sorted.end());
        size_t q_idx = static_cast<size_t>(std::ceil((1.0 - alpha_current) * sorted.size()));
        q_idx = std::clamp(q_idx, size_t(0), sorted.size() - 1);
        current_quantile = sorted[q_idx];

        double err_k = (s_k > current_quantile) ? 1.0 : 0.0;
        alpha_current += gamma_lr * (alpha_target - err_k);
        alpha_current = std::clamp(alpha_current, 0.001, 0.30);

        if (s_k > current_quantile && current_quantile > 1e-6) {
            current_inflation = std::pow(s_k / current_quantile, 2.0);
        } else {
            current_inflation = 1.0;
        }
        return current_inflation;
    }
};

class TunedConESKF {
public:
    TunedConESKF() {
        gravity_ = Eigen::Vector3d(0, 0, -9.80665);
        state_ = State15();
        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 0.1;
        P_.block<3, 3>(3, 3) *= 0.1;
        P_.block<3, 3>(6, 6) *= 0.01;
        P_.block<3, 3>(9, 9) *= 0.001;
        P_.block<3, 3>(12, 12) *= 0.0001;

        acc_noise_ = 0.08; gyr_noise_ = 0.004;
        acc_bias_noise_ = 0.0004; gyr_bias_noise_ = 0.00003;
    }

    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) {
        state_.p = p0; state_.R = R0; state_.v.setZero();
    }

    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) {
        Eigen::Vector3d acc = a_m - state_.ba;
        Eigen::Vector3d gyr = w_m - state_.bg;

        Eigen::Matrix3d dR = coneskf::expSO3(gyr * dt);
        Eigen::Vector3d acc_world = state_.R * acc + gravity_;

        state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
        state_.v += acc_world * dt;
        state_.R = state_.R * dR;

        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * state_.R * coneskf::skew(acc) * (dt * dt);
        F.block<3, 3>(0, 9) = -0.5 * state_.R * (dt * dt);
        F.block<3, 3>(3, 6) = -state_.R * coneskf::skew(acc) * dt;
        F.block<3, 3>(3, 9) = -state_.R * dt;
        F.block<3, 3>(6, 6) = coneskf::expSO3(-gyr * dt);
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
        Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt * dt * dt / 3.0);
        Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt);
        Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (gyr_noise_ * gyr_noise_ * dt);
        Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (acc_bias_noise_ * acc_bias_noise_ * dt);
        Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (gyr_bias_noise_ * gyr_bias_noise_ * dt);

        P_ = F * P_ * F.transpose() + Qd;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov, const std::string& modality_id) {
        if (martingales_.find(modality_id) == martingales_.end()) {
            martingales_[modality_id] = TunedMartingale();
            aci_[modality_id] = ACIChan();
        }
        auto& mart = martingales_[modality_id];
        auto& aci = aci_[modality_id];

        Eigen::Vector3d nu = z_pos - state_.p;
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S = H * P_ * H.transpose() + R_cov;
        Eigen::LLT<Eigen::Matrix3d> llt(S);
        if (llt.info() != Eigen::Success) return false;

        double s_k = llt.matrixL().solve(nu).norm();
        mart.update(s_k);

        if (mart.isQuarantined()) return false;

        double phi_k = aci.update(s_k);
        Eigen::Matrix3d R_tilde = phi_k * R_cov;
        Eigen::Matrix3d S_tilde = H * P_ * H.transpose() + R_tilde;
        Eigen::LLT<Eigen::Matrix3d> lltT(S_tilde);
        if (lltT.info() != Eigen::Success) return false;

        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
        K = lltT.solve(K.transpose()).transpose();

        Eigen::Matrix<double, 15, 1> delta_x = K * nu;
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_tilde * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        state_.p += delta_x.segment<3>(0);
        state_.v += delta_x.segment<3>(3);
        state_.R = state_.R * coneskf::expSO3(delta_x.segment<3>(6));
        state_.ba += delta_x.segment<3>(9);
        state_.bg += delta_x.segment<3>(12);

        return true;
    }

    const State15& getState() const { return state_; }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const { return P_; }
    bool isQuarantined(const std::string& mod) {
        if (martingales_.find(mod) != martingales_.end()) return martingales_[mod].isQuarantined();
        return false;
    }
    double getWealth(const std::string& mod) {
        if (martingales_.find(mod) != martingales_.end()) return martingales_[mod].getWealth();
        return 1.0;
    }

private:
    State15 state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Vector3d gravity_;
    double acc_noise_, gyr_noise_, acc_bias_noise_, gyr_bias_noise_;
    std::map<std::string, TunedMartingale> martingales_;
    std::map<std::string, ACIChan> aci_;
};

int main() {
    std::string imu_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_imu.csv";
    std::string leica_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_leica.csv";

    std::vector<std::pair<double, Eigen::Vector3d>> imu_a, imu_w;
    std::vector<std::pair<double, Eigen::Vector3d>> leica_p;

    std::ifstream fi(imu_csv);
    std::string line;
    std::getline(fi, line);
    while (std::getline(fi, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string v;
        double t, wx, wy, wz, ax, ay, az;
        std::getline(ss, v, ','); t = std::stod(v);
        std::getline(ss, v, ','); wx = std::stod(v);
        std::getline(ss, v, ','); wy = std::stod(v);
        std::getline(ss, v, ','); wz = std::stod(v);
        std::getline(ss, v, ','); ax = std::stod(v);
        std::getline(ss, v, ','); ay = std::stod(v);
        std::getline(ss, v, ','); az = std::stod(v);
        imu_a.push_back({t, Eigen::Vector3d(ax, ay, az)});
        imu_w.push_back({t, Eigen::Vector3d(wx, wy, wz)});
    }

    std::ifstream fl(leica_csv);
    std::getline(fl, line);
    while (std::getline(fl, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string v;
        double t, px, py, pz;
        std::getline(ss, v, ','); t = std::stod(v);
        std::getline(ss, v, ','); px = std::stod(v);
        std::getline(ss, v, ','); py = std::stod(v);
        std::getline(ss, v, ','); pz = std::stod(v);
        leica_p.push_back({t, Eigen::Vector3d(px, py, pz)});
    }

    Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < 50; ++k) a_sum += imu_a[k].second;
    Eigen::Matrix3d R0 = coneskf::computeGravityAlignment(a_sum / 50.0);

    std::vector<std::string> scenarios = {"S0_CleanFlight", "S1_HeavyTailNLoS", "S2_StealthRamp"};

    for (const auto& scen : scenarios) {
        TunedConESKF filter;
        filter.initialize(leica_p[0].second, R0);
        coneskf::CertifiedPoseTube tube(0.05, 120);

        std::mt19937_64 rng(42);
        std::uniform_real_distribution<double> unif(0.0, 1.0);
        std::normal_distribution<double> norm_dist(0.0, 0.15);
        std::cauchy_distribution<double> cauchy_dist(0.0, 2.5);

        Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity() * 0.04;
        Eigen::Matrix3d R_uwb = Eigen::Matrix3d::Identity() * 0.09;

        size_t l_idx = 0;
        std::vector<double> pos_errors;
        double attack_onset_t = 30.0;
        double detect_t = -1.0;

        for (size_t i = 1; i < imu_a.size(); ++i) {
            double dt = imu_a[i].first - imu_a[i-1].first;
            if (dt <= 0.0 || dt > 0.1) dt = 0.005;
            filter.predictIMU(imu_a[i].second, imu_w[i].second, dt);

            double t = imu_a[i].first;
            while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
                Eigen::Vector3d p_true = leica_p[l_idx].second;
                Eigen::Vector3d z_gnss = p_true;

                if (scen == "S1_HeavyTailNLoS") {
                    if (unif(rng) < 0.15) {
                        z_gnss.x() += cauchy_dist(rng);
                        z_gnss.y() += cauchy_dist(rng);
                        z_gnss.z() += 0.5 * cauchy_dist(rng);
                    }
                } else if (scen == "S2_StealthRamp") {
                    if (t >= attack_onset_t) {
                        double dt_att = t - attack_onset_t;
                        z_gnss.x() += 0.15 * dt_att;
                        z_gnss.y() += 0.10 * dt_att;
                    }
                }

                filter.updatePosition(z_gnss, R_gnss, "gnss");

                if (filter.isQuarantined("gnss") && detect_t < 0.0) {
                    detect_t = t;
                }

                if (scen == "S2_StealthRamp") {
                    Eigen::Vector3d z_uwb = p_true;
                    z_uwb.x() += norm_dist(rng);
                    z_uwb.y() += norm_dist(rng);
                    z_uwb.z() += norm_dist(rng);
                    filter.updatePosition(z_uwb, R_uwb, "uwb");
                }

                double err = (filter.getState().p - p_true).norm();
                pos_errors.push_back(err);
                tube.update(filter.getState().p, filter.getCovariance().block<3, 3>(0, 0), p_true);

                l_idx++;
            }
        }

        double sum_sq = 0.0, max_e = 0.0;
        for (double e : pos_errors) {
            sum_sq += e * e;
            if (e > max_e) max_e = e;
        }
        double rmse = std::sqrt(sum_sq / pos_errors.size());
        double ttd = (detect_t >= 0.0) ? (detect_t - attack_onset_t) : -1.0;
        bool fa = (detect_t >= 0.0 && detect_t < attack_onset_t);

                std::cout << "[" << scen << "] Proposed ConESKF-FDI: "
                  << "RMSE: " << rmse << " m, "
                  << "Max: " << max_e << " m, "
                  << "TubeCov: " << tube.getEmpiricalCoverage() * 100.0 << " %, "
                  << "TTD: " << (ttd >= 0 ? std::to_string(ttd) + " s" : "None") << ", "
                  << "FalseAlarm: " << (fa ? "TRUE" : "FALSE") << ", "
                  << "Wealth: " << filter.getWealth("gnss") << std::endl;
    }
    return 0;
}
