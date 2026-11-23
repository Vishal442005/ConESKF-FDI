
#include "coneskf_core/baselines.hpp"
#include <iostream>
#include <fstream>
#include <vector>

void testTuning(size_t window, size_t min_calib, double eps, double lambda) {
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

    // Initialize ESKF
    coneskf::ConESKFFilter filter;
    Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < 50; ++k) a_sum += imu_a[k].second;
    Eigen::Matrix3d R0 = coneskf::computeGravityAlignment(a_sum / 50.0);
    filter.initialize(leica_p[0].second, R0);

    // Custom martingale
    coneskf::ConformalBettingMartingale mart(window, min_calib, eps, lambda, 0.01, 42);

    Eigen::Matrix3d R_cov = Eigen::Matrix3d::Identity() * 0.04;
    size_t l_idx = 0;
    double max_wealth = 0.0;
    size_t false_alarms = 0;
    double first_alarm_t = -1.0;

    for (size_t i = 1; i < imu_a.size(); ++i) {
        double dt = imu_a[i].first - imu_a[i-1].first;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;
        filter.predictIMU(imu_a[i].second, imu_w[i].second, dt);

        double t = imu_a[i].first;
        while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
            // Compute whitened norm
            Eigen::Vector3d nu = leica_p[l_idx].second - filter.getState().p;
            Eigen::Matrix3d S = filter.getCovariance().block<3, 3>(0, 0) + R_cov;
            Eigen::LLT<Eigen::Matrix3d> llt(S);
            double s_k = llt.matrixL().solve(nu).norm();

            double w = mart.update(s_k);
            if (w > max_wealth) max_wealth = w;
            if (mart.isQuarantined()) {
                if (false_alarms == 0) first_alarm_t = t;
                false_alarms++;
            }

            // Normal update without quarantine for baseline tracking
            filter.updatePosition(leica_p[l_idx].second, R_cov, "gnss");
            l_idx++;
        }
    }
    std::cout << "Config [win=" << window << ", min_cal=" << min_calib << ", eps=" << eps 
              << ", lam=" << lambda << "] -> MaxWealth: " << max_wealth 
              << ", Alarms: " << false_alarms 
              << (first_alarm_t > 0 ? (" at t=" + std::to_string(first_alarm_t)) : "") << std::endl;
}

int main() {
    testTuning(120, 40, 0.35, 0.02);
    testTuning(150, 80, 0.20, 0.03);
    testTuning(200, 100, 0.18, 0.04);
    testTuning(200, 150, 0.15, 0.05);
    testTuning(250, 150, 0.15, 0.05);
    testTuning(300, 200, 0.12, 0.05);
    return 0;
}
