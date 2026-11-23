
#include "coneskf_core/baselines.hpp"
#include <iostream>
#include <fstream>
#include <vector>

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

    // Test on S1 (Heavy-Tail NLoS multipath)
    coneskf::StandardChi2ESKF eskf;
    eskf.initialize(leica_p[0].second, R0);
    Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity() * 0.04;

    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    std::cauchy_distribution<double> cauchy_dist(0.0, 2.5);

    std::deque<double> history;
    double wealth = 1.0;
    double max_wealth = 0.0;
    size_t fa = 0;
    size_t l_idx = 0;

    for (size_t i = 1; i < imu_a.size(); ++i) {
        double dt = imu_a[i].first - imu_a[i-1].first;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;
        eskf.predictIMU(imu_a[i].second, imu_w[i].second, dt);

        double t = imu_a[i].first;
        while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
            Eigen::Vector3d p_true = leica_p[l_idx].second;
            Eigen::Vector3d z_gnss = p_true;

            // Scenario 1: Heavy-tail NLoS spikes (15% burst rate)
            if (unif(rng) < 0.15) {
                z_gnss.x() += cauchy_dist(rng);
                z_gnss.y() += cauchy_dist(rng);
                z_gnss.z() += 0.5 * cauchy_dist(rng);
            }

            Eigen::Vector3d nu = z_gnss - eskf.getState().p;
            Eigen::Matrix3d S = eskf.getCovariance().block<3, 3>(0, 0) + R_gnss;
            Eigen::LLT<Eigen::Matrix3d> llt(S);
            double s_k = llt.matrixL().solve(nu).norm();

            if (history.size() >= 100) {
                size_t count_greater = 0, count_equal = 0;
                for (double s_past : history) {
                    if (s_past > s_k) count_greater++;
                    else if (std::abs(s_past - s_k) < 1e-9) count_equal++;
                }
                double u_k = unif(rng);
                double p_k = (static_cast<double>(count_greater) + u_k * (count_equal + 1.0)) / (history.size() + 1.0);
                p_k = std::clamp(p_k, 1e-6, 1.0 - 1e-6);

                double e_k = 1.0 + 0.16 * (1.0 - 2.0 * p_k);
                wealth = wealth * e_k;
                if (wealth > max_wealth) max_wealth = wealth;
                if (wealth >= 100.0) fa++;
            }

            history.push_back(s_k);
            if (history.size() > 200) history.pop_front();

            eskf.updatePosition(z_gnss, R_gnss);
            l_idx++;
        }
    }

    std::cout << "S1 (Heavy-Tail NLoS) Pure Martingale (eps=0.16, win=200, min_cal=100):" << std::endl;
    std::cout << "Max Wealth: " << max_wealth << ", Alarms (wealth >= 100): " << fa << std::endl;

    return 0;
}
