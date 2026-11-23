#include "coneskf_core/coneskf.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

int main() {
    coneskf::ConESKFFilter filter;
    std::ifstream f_imu("/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_imu.csv");
    std::ifstream f_lei("/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_leica.csv");
    std::string line;
    std::getline(f_imu, line);
    std::getline(f_lei, line);

    std::vector<std::pair<double, Eigen::Vector3d>> leica;
    while (std::getline(f_lei, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string v;
        double t, x, y, z;
        std::getline(ss, v, ','); t = std::stod(v);
        std::getline(ss, v, ','); x = std::stod(v);
        std::getline(ss, v, ','); y = std::stod(v);
        std::getline(ss, v, ','); z = std::stod(v);
        leica.push_back({t, Eigen::Vector3d(x, y, z)});
    }

    // Initialize with first Leica
    filter.initialize(leica[0].second, Eigen::Matrix3d::Identity());

    size_t l_idx = 0;
    double t_prev = 0.0;
    double s_k = 0, phi_k = 0, m_k = 0;

    while (std::getline(f_imu, line)) {
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

        double dt = (t_prev > 0) ? (t - t_prev) : 0.005;
        t_prev = t;

        filter.predictIMU(Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(wx, wy, wz), dt);

        while (l_idx < leica.size() && leica[l_idx].first <= t) {
            Eigen::Vector3d z = leica[l_idx].second;
            // Inject ramp starting at t=30
            if (t >= 30.0) {
                z.x() += 0.15 * (t - 30.0);
            }
            filter.updatePosition(z, Eigen::Matrix3d::Identity() * 0.04, "gps", &s_k, &phi_k, &m_k);
            if (static_cast<int>(t * 10) % 20 == 0) {
                std::cout << "t=" << t << " s: s_k=" << s_k << ", phi=" << phi_k << ", wealth M_k=" << m_k << std::endl;
            }
            l_idx++;
        }
    }
    return 0;
}
