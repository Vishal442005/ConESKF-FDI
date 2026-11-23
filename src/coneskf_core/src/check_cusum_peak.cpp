
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

    coneskf::StandardChi2ESKF eskf;
    eskf.initialize(leica_p[0].second, R0);
    Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity() * 0.04;

    std::deque<double> history;
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    double g_cusum = 0.0;
    double max_g = 0.0;
    size_t first_over_h = 0;
    double first_t = -1.0;
    double drift = 0.02;

    size_t l_idx = 0;
    for (size_t i = 1; i < imu_a.size(); ++i) {
        double dt = imu_a[i].first - imu_a[i-1].first;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;
        eskf.predictIMU(imu_a[i].second, imu_w[i].second, dt);

        double t = imu_a[i].first;
        while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
            Eigen::Vector3d nu = leica_p[l_idx].second - eskf.getState().p;
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

                double e_k = 1.0 + 0.20 * (1.0 - 2.0 * p_k);
                g_cusum = std::max(0.0, g_cusum + std::log(e_k) - drift);
                if (g_cusum > max_g) max_g = g_cusum;

                if (g_cusum >= 4.605 && first_t < 0.0) {
                    first_t = t;
                    first_over_h = l_idx;
                }
            }

            history.push_back(s_k);
            if (history.size() > 200) history.pop_front();

            eskf.updatePosition(leica_p[l_idx].second, R_gnss);
            l_idx++;
        }
    }

    std::cout << "Pure closed-loop CUSUM (eps=0.20, drift=" << drift << "):" << std::endl;
    std::cout << "Max g over entire flight: " << max_g << std::endl;
    if (first_t > 0) {
        std::cout << "First crossed 4.605 at t = " << first_t << " (l_idx=" << first_over_h << ")" << std::endl;
    } else {
        std::cout << "NEVER crossed 4.605! (Zero false alarms)" << std::endl;
    }

    return 0;
}
