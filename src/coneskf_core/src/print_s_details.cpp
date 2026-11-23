
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
    Eigen::Matrix3d R_uwb = Eigen::Matrix3d::Identity() * 0.09;
    size_t l_idx = 0;

    std::mt19937_64 rng(42);
    std::normal_distribution<double> norm_dist(0.0, 0.15);

    std::vector<double> s_gnss_hist;

    for (size_t i = 1; i < imu_a.size(); ++i) {
        double dt = imu_a[i].first - imu_a[i-1].first;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;
        eskf.predictIMU(imu_a[i].second, imu_w[i].second, dt);

        double t = imu_a[i].first;
        while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
            Eigen::Vector3d p_true = leica_p[l_idx].second;
            Eigen::Vector3d z_gnss = p_true;

            Eigen::Vector3d nu = z_gnss - eskf.getState().p;
            Eigen::Matrix3d S = eskf.getCovariance().block<3, 3>(0, 0) + R_gnss;
            Eigen::LLT<Eigen::Matrix3d> llt(S);
            double s_k = llt.matrixL().solve(nu).norm();
            s_gnss_hist.push_back(s_k);

            eskf.updatePosition(z_gnss, R_gnss);

            Eigen::Vector3d z_uwb = p_true;
            z_uwb.x() += norm_dist(rng);
            z_uwb.y() += norm_dist(rng);
            z_uwb.z() += norm_dist(rng);
            eskf.updatePosition(z_uwb, R_uwb);

            if (t >= 24.5 && t <= 26.0) {
                std::cout << "t=" << t << " s_k=" << s_k << " nu_norm=" << nu.norm() 
                          << " S_diag=" << S(0,0) << std::endl;
            }

            l_idx++;
        }
        if (t > 26.5) break;
    }
    return 0;
}
