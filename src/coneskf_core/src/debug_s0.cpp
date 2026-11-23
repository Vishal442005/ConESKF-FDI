#include "coneskf_core/baselines.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

int main() {
    auto imu = loadIMU("/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_imu.csv");
    auto leica = loadLeica("/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_leica.csv");

    coneskf::ConESKFFilter filter;
    Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < 50; ++k) a_sum += imu[k].a;
    Eigen::Matrix3d R0 = coneskf::computeGravityAlignment(a_sum / 50.0);
    filter.initialize(leica[0].p, R0);

    size_t l_idx = 0;
    double s_k = 0, phi_k = 0, m_k = 0;

    for (size_t i = 1; i < imu.size(); ++i) {
        double dt = imu[i].t - imu[i-1].t;
        if (dt <= 0 || dt > 0.1) dt = 0.005;
        filter.predictIMU(imu[i].a, imu[i].w, dt);

        double t = imu[i].t;
        while (l_idx < leica.size() && leica[l_idx].t <= t) {
            filter.updatePosition(leica[l_idx].p, Eigen::Matrix3d::Identity() * 0.04, "gnss", &s_k, &phi_k, &m_k);
            if (l_idx < 60 || l_idx % 50 == 0) {
                std::cout << "Leica #" << l_idx << " (t=" << t << "s): s_k=" << s_k 
                          << ", phi=" << phi_k << ", M_k=" << m_k << std::endl;
            }
            if (filter.isChannelQuarantined("gnss")) {
                std::cout << ">>> QUARANTINE TRIGGERED at Leica #" << l_idx << " (t=" << t 
                          << "s) with M_k=" << m_k << "! <<<" << std::endl;
                return 0;
            }
            l_idx++;
        }
    }
    std::cout << "Flight completed without quarantine." << std::endl;
    return 0;
}
