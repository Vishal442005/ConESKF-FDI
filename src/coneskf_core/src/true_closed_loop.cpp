
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

    // Standard ESKF baseline (always updates, never quarantines)
    coneskf::StandardChi2ESKF eskf;
    Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < 50; ++k) a_sum += imu_a[k].second;
    Eigen::Matrix3d R0 = coneskf::computeGravityAlignment(a_sum / 50.0);
    eskf.initialize(leica_p[0].second, R0);

    Eigen::Matrix3d R_cov = Eigen::Matrix3d::Identity() * 0.04;
    size_t l_idx = 0;

    std::vector<double> s_values;
    std::vector<double> err_values;

    for (size_t i = 1; i < imu_a.size(); ++i) {
        double dt = imu_a[i].first - imu_a[i-1].first;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;
        eskf.predictIMU(imu_a[i].second, imu_w[i].second, dt);

        double t = imu_a[i].first;
        while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
            Eigen::Vector3d nu = leica_p[l_idx].second - eskf.getState().p;
            Eigen::Matrix3d S = eskf.getCovariance().block<3, 3>(0, 0) + R_cov;
            Eigen::LLT<Eigen::Matrix3d> llt(S);
            double s_k = llt.matrixL().solve(nu).norm();
            double err = nu.norm();

            s_values.push_back(s_k);
            err_values.push_back(err);

            eskf.updatePosition(leica_p[l_idx].second, R_cov);
            l_idx++;
        }
    }

    double sum_s = 0, max_s = 0, min_s = 1e9;
    double sum_e = 0, max_e = 0;
    for (size_t k = 0; k < s_values.size(); ++k) {
        sum_s += s_values[k];
        if (s_values[k] > max_s) max_s = s_values[k];
        if (s_values[k] < min_s) min_s = s_values[k];
        sum_e += err_values[k];
        if (err_values[k] > max_e) max_e = err_values[k];
    }
    std::cout << "Closed-loop across all " << s_values.size() << " steps:" << std::endl;
    std::cout << "s_k: mean = " << sum_s / s_values.size() << ", min = " << min_s << ", max = " << max_s << std::endl;
    std::cout << "err: mean = " << sum_e / err_values.size() << ", max = " << max_e << std::endl;

    return 0;
}
