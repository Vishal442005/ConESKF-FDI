
#include "coneskf_core/baselines.hpp"
#include "coneskf_core/pose_tube.hpp"
#include <iostream>
#include <fstream>
#include <vector>

void evaluateEstimator(const std::string& scen_name, double eps, size_t win, size_t min_cal) {
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

    coneskf::ConESKFFilter filter;
    filter.initialize(leica_p[0].second, R0);

    // Overwrite the martingale in filter with tuned params
    coneskf::ConformalBettingMartingale mart(win, min_cal, eps, 0.0, 0.01, 42);
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
    bool is_gnss_quarantined = false;

    for (size_t i = 1; i < imu_a.size(); ++i) {
        double dt = imu_a[i].first - imu_a[i-1].first;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;
        filter.predictIMU(imu_a[i].second, imu_w[i].second, dt);

        double t = imu_a[i].first;
        while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
            Eigen::Vector3d p_true = leica_p[l_idx].second;
            Eigen::Vector3d z_gnss = p_true;

            if (scen_name == "S1_HeavyTailNLoS") {
                if (unif(rng) < 0.15) {
                    z_gnss.x() += cauchy_dist(rng);
                    z_gnss.y() += cauchy_dist(rng);
                    z_gnss.z() += 0.5 * cauchy_dist(rng);
                }
            } else if (scen_name == "S2_StealthRamp") {
                if (t >= attack_onset_t) {
                    double dt_att = t - attack_onset_t;
                    z_gnss.x() += 0.15 * dt_att;
                    z_gnss.y() += 0.10 * dt_att;
                }
            }

            // Test GNSS channel
            if (!is_gnss_quarantined) {
                Eigen::Vector3d nu = z_gnss - filter.getState().p;
                Eigen::Matrix3d S = filter.getCovariance().block<3, 3>(0, 0) + R_gnss;
                Eigen::LLT<Eigen::Matrix3d> llt(S);
                if (llt.info() == Eigen::Success) {
                    double s_k = llt.matrixL().solve(nu).norm();
                    double w = mart.update(s_k);
                    if (mart.isQuarantined()) {
                        is_gnss_quarantined = true;
                        if (detect_t < 0.0) detect_t = t;
                    }
                }
            }

            // Update filter
            if (!is_gnss_quarantined) {
                filter.updatePosition(z_gnss, R_gnss, "gnss");
            }

            // Secondary channel in S2
            if (scen_name == "S2_StealthRamp") {
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
    double ttd = (detect_t > 0.0) ? (detect_t - attack_onset_t) : -1.0;
    bool fa = (detect_t > 0.0 && detect_t < attack_onset_t);

    std::cout << "[" << scen_name << "] Proposed ConESKF-FDI -> ATE RMSE: " << rmse 
              << " m, Max: " << max_e << " m, Tube Coverage: " << tube.getEmpiricalCoverage() * 100.0 
              << "%, TTD: " << ttd << " s, FalseAlarm: " << (fa ? "TRUE" : "FALSE") 
              << ", MaxWealth: " << mart.getWealth() << std::endl;
}

int main() {
    evaluateEstimator("S0_CleanFlight", 0.16, 200, 100);
    evaluateEstimator("S1_HeavyTailNLoS", 0.16, 200, 100);
    evaluateEstimator("S2_StealthRamp", 0.16, 200, 100);
    return 0;
}
