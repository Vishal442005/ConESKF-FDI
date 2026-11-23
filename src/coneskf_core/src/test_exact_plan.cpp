
#include "coneskf_core/coneskf.hpp"
#include "coneskf_core/pose_tube.hpp"
#include <Eigen/Dense>
#include <deque>
#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>

class ExactPlanMartingale {
public:
    ExactPlanMartingale(
        size_t window_size = 200,
        size_t min_calibration_samples = 100,
        double epsilon_bet = 0.20,
        double alpha_far = 0.001, // 10^-3 as in plan paragraph 100
        uint64_t seed = 42
    ) : window_size_(window_size),
        min_calibration_samples_(min_calibration_samples),
        epsilon_bet_(epsilon_bet),
        alpha_far_(alpha_far),
        threshold_(1.0 / alpha_far), // 1000.0
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

        // Bounded betting e-factor: e_k = 1 + eps * (1 - 2 * p_k)
        double e_k = 1.0 + epsilon_bet_ * (1.0 - 2.0 * p_k);

        // Shiryaev-Roberts / Martingale: M_k = max(1.0, M_{k-1}) * e_k (Paragraph 99)
        wealth_ = std::max(1.0, wealth_) * e_k;

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
    double getThreshold() const { return threshold_; }

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

// Test on real EuRoC data with this exact plan martingale
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

    auto runScen = [&](const std::string& scen, double eps, size_t win, size_t min_cal, double alpha_far) {
        ExactPlanMartingale mart(win, min_cal, eps, alpha_far, 42);

        // Simple nominal ESKF
        coneskf::State15 state;
        state.p = leica_p[0].second; state.R = R0; state.v.setZero();
        Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;
        P.block<3, 3>(6, 6) *= 0.1; P.block<3, 3>(9, 9) *= 0.01; P.block<3, 3>(12, 12) *= 0.001;

        Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity() * 0.04;
        Eigen::Matrix3d R_uwb = Eigen::Matrix3d::Identity() * 0.09;

        std::mt19937_64 rng(42);
        std::uniform_real_distribution<double> unif(0.0, 1.0);
        std::normal_distribution<double> norm_dist(0.0, 0.15);
        std::cauchy_distribution<double> cauchy_dist(0.0, 2.5);

        size_t l_idx = 0;
        double attack_onset_t = 30.0;
        double detect_t = -1.0;
        double max_wealth = 0.0;
        std::vector<double> errors;

        for (size_t i = 1; i < imu_a.size(); ++i) {
            double dt = imu_a[i].first - imu_a[i-1].first;
            if (dt <= 0.0 || dt > 0.1) dt = 0.005;

            // IMU predict
            Eigen::Vector3d acc = imu_a[i].second - state.ba;
            Eigen::Vector3d gyr = imu_w[i].second - state.bg;
            Eigen::Matrix3d dR = coneskf::expSO3(gyr * dt);
            Eigen::Vector3d acc_w = state.R * acc + Eigen::Vector3d(0, 0, -9.80665);
            state.p += state.v * dt + 0.5 * acc_w * dt * dt;
            state.v += acc_w * dt;
            state.R = state.R * dR;

            Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
            F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
            F.block<3, 3>(0, 6) = -0.5 * state.R * coneskf::skew(acc) * (dt * dt);
            F.block<3, 3>(0, 9) = -0.5 * state.R * (dt * dt);
            F.block<3, 3>(3, 6) = -state.R * coneskf::skew(acc) * dt;
            F.block<3, 3>(3, 9) = -state.R * dt;
            F.block<3, 3>(6, 6) = coneskf::expSO3(-gyr * dt);
            F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

            Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
            Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.08 * 0.08 * dt * dt * dt / 3.0);
            Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (0.08 * 0.08 * dt);
            Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (0.004 * 0.004 * dt);
            Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (0.0004 * 0.0004 * dt);
            Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (0.00003 * 0.00003 * dt);
            P = F * P * F.transpose() + Qd;
            P = 0.5 * (P + P.transpose());

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

                Eigen::Vector3d nu = z_gnss - state.p;
                Eigen::Matrix3d S = P.block<3, 3>(0, 0) + R_gnss;
                Eigen::LLT<Eigen::Matrix3d> llt(S);
                double s_k = llt.matrixL().solve(nu).norm();

                double w = mart.update(s_k);
                if (w > max_wealth) max_wealth = w;

                if (mart.isQuarantined() && detect_t < 0.0) {
                    detect_t = t;
                }

                // Update GNSS if not quarantined
                if (!mart.isQuarantined()) {
                    Eigen::Matrix<double, 15, 3> H_t = Eigen::Matrix<double, 15, 3>::Zero();
                    H_t.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                    Eigen::Matrix<double, 15, 3> K = P * H_t;
                    K = llt.solve(K.transpose()).transpose();
                    Eigen::Matrix<double, 15, 1> dx = K * nu;
                    Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity();
                    I_KH.block<15, 3>(0, 0) -= K;
                    P = I_KH * P * I_KH.transpose() + K * R_gnss * K.transpose();
                    P = 0.5 * (P + P.transpose());
                    state.p += dx.segment<3>(0);
                    state.v += dx.segment<3>(3);
                    state.R = state.R * coneskf::expSO3(dx.segment<3>(6));
                    state.ba += dx.segment<3>(9);
                    state.bg += dx.segment<3>(12);
                }

                // In S2, secondary UWB aiding channel keeps estimator anchored after GNSS quarantine!
                if (scen == "S2_StealthRamp") {
                    Eigen::Vector3d z_uwb = p_true;
                    z_uwb.x() += norm_dist(rng);
                    z_uwb.y() += norm_dist(rng);
                    z_uwb.z() += norm_dist(rng);

                    Eigen::Vector3d nu_u = z_uwb - state.p;
                    Eigen::Matrix3d S_u = P.block<3, 3>(0, 0) + R_uwb;
                    Eigen::LLT<Eigen::Matrix3d> llt_u(S_u);
                    Eigen::Matrix<double, 15, 3> H_t = Eigen::Matrix<double, 15, 3>::Zero();
                    H_t.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                    Eigen::Matrix<double, 15, 3> K_u = P * H_t;
                    K_u = llt_u.solve(K_u.transpose()).transpose();
                    Eigen::Matrix<double, 15, 1> dx_u = K_u * nu_u;
                    Eigen::Matrix<double, 15, 15> I_KH_u = Eigen::Matrix<double, 15, 15>::Identity();
                    I_KH_u.block<15, 3>(0, 0) -= K_u;
                    P = I_KH_u * P * I_KH_u.transpose() + K_u * R_uwb * K_u.transpose();
                    P = 0.5 * (P + P.transpose());
                    state.p += dx_u.segment<3>(0);
                    state.v += dx_u.segment<3>(3);
                    state.R = state.R * coneskf::expSO3(dx_u.segment<3>(6));
                    state.ba += dx_u.segment<3>(9);
                    state.bg += dx_u.segment<3>(12);
                }

                errors.push_back((state.p - p_true).norm());
                l_idx++;
            }
        }

        double sum_sq = 0, max_e = 0;
        for (double e : errors) {
            sum_sq += e * e;
            if (e > max_e) max_e = e;
        }
        double rmse = std::sqrt(sum_sq / errors.size());
        double ttd = (detect_t >= 0.0) ? (detect_t - attack_onset_t) : -1.0;
        bool fa = (detect_t >= 0.0 && detect_t < attack_onset_t);

        std::cout << "[" << scen << "] eps=" << eps << ", win=" << win << " -> RMSE: " << rmse 
                  << " m, MaxErr: " << max_e << " m, MaxWealth: " << max_wealth 
                  << ", TTD: " << (ttd >= 0 ? std::to_string(ttd) + "s" : "None") 
                  << ", FalseAlarm: " << (fa ? "TRUE" : "FALSE") << std::endl;
    };

    std::cout << "--- Testing with alpha_far = 0.001 (Threshold = 1000) ---" << std::endl;
    runScen("S0_CleanFlight", 0.20, 200, 100, 0.001);
    runScen("S1_HeavyTailNLoS", 0.20, 200, 100, 0.001);
    runScen("S2_StealthRamp", 0.20, 200, 100, 0.001);

    std::cout << "\n--- Testing with alpha_far = 0.005 (Threshold = 200) ---\n";
    runScen("S0_CleanFlight", 0.18, 200, 100, 0.005);
    runScen("S1_HeavyTailNLoS", 0.18, 200, 100, 0.005);
    runScen("S2_StealthRamp", 0.18, 200, 100, 0.005);

    return 0;
}
