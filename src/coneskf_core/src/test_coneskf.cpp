#include "coneskf_core/coneskf.hpp"
#include <chrono>
#include <iostream>
#include <vector>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "ConESKF-FDI C++17 Core Engine Benchmark" << std::endl;
    std::cout << "========================================" << std::endl;

    coneskf::ConESKFFilter filter;
    filter.initialize(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

    Eigen::Vector3d a_m(0.05, -0.02, 9.81);
    Eigen::Vector3d w_m(0.01, 0.005, -0.002);
    double dt = 0.005; // 200 Hz

    Eigen::Vector3d z_pos(0.1, 0.2, -0.05);
    Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * 0.04;

    const int N_TICKS = 2000; // 10 seconds of 200 Hz flight
    std::vector<double> latencies_us;
    latencies_us.reserve(N_TICKS);

    double s_k = 0.0, phi_k = 0.0, m_k = 0.0;

    for (int i = 0; i < N_TICKS; ++i) {
        auto t_start = std::chrono::high_resolution_clock::now();

        // 1. IMU Propagation (200 Hz)
        filter.predictIMU(a_m, w_m, dt);

        // 2. Aided measurement update at 20 Hz (every 10 ticks)
        if (i % 10 == 0) {
            filter.updatePosition(z_pos, R_pos, "vicon", &s_k, &phi_k, &m_k);
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        double us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        latencies_us.push_back(us);
    }

    // Compute latency stats
    double sum = 0.0;
    double max_us = 0.0;
    for (double us : latencies_us) {
        sum += us;
        if (us > max_us) max_us = us;
    }
    double mean_us = sum / latencies_us.size();

    std::sort(latencies_us.begin(), latencies_us.end());
    double p50_us = latencies_us[latencies_us.size() / 2];
    double p99_us = latencies_us[static_cast<size_t>(latencies_us.size() * 0.99)];

    std::cout << "Executed " << N_TICKS << " filter ticks successfully." << std::endl;
    std::cout << "State estimate: p=[" << filter.getState().p.transpose() 
              << "], v=[" << filter.getState().v.transpose() << "]" << std::endl;
    std::cout << "Mean Latency:    " << mean_us << " us (" << mean_us / 1000.0 << " ms)" << std::endl;
    std::cout << "P50 Latency:     " << p50_us << " us (" << p50_us / 1000.0 << " ms)" << std::endl;
    std::cout << "P99 Latency:     " << p99_us << " us (" << p99_us / 1000.0 << " ms)" << std::endl;
    std::cout << "Max Latency:     " << max_us << " us (" << max_us / 1000.0 << " ms)" << std::endl;
    std::cout << "200 Hz Deadline: 5000.0 us (5.0 ms)" << std::endl;

    if (mean_us < 150.0) {
        std::cout << ">> VERIFICATION PASSED: Latency is << 0.15 ms target!" << std::endl;
    } else {
        std::cout << ">> Target exceeded: " << mean_us << " us." << std::endl;
    }

    return 0;
}
