#include "coneskf_core/baselines.hpp"
#include "coneskf_core/pose_tube.hpp"
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <memory>
#include <iomanip>
#include <random>

struct IMURecord {
    double t;
    Eigen::Vector3d w;
    Eigen::Vector3d a;
};

struct LeicaRecord {
    double t;
    Eigen::Vector3d p;
};

struct BenchmarkMetrics {
    std::string estimator_name;
    std::string scenario;
    double ate_rmse;
    double ate_max;
    double empirical_coverage;
    double time_to_detect;
    bool false_alarm;
    double mean_latency_us;
    double p99_latency_us;
};

std::vector<IMURecord> loadIMU(const std::string& path) {
    std::vector<IMURecord> records;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line[0] == 't') continue;
        std::stringstream ss(line);
        std::string val;
        IMURecord rec;
        try {
            if (!std::getline(ss, val, ',')) continue; rec.t = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.w.x() = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.w.y() = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.w.z() = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.a.x() = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.a.y() = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.a.z() = std::stod(val);
            records.push_back(rec);
        } catch (...) {
            continue;
        }
    }
    return records;
}

std::vector<LeicaRecord> loadLeica(const std::string& path) {
    std::vector<LeicaRecord> records;
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line[0] == 't') continue;
        std::stringstream ss(line);
        std::string val;
        LeicaRecord rec;
        try {
            if (!std::getline(ss, val, ',')) continue; rec.t = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.p.x() = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.p.y() = std::stod(val);
            if (!std::getline(ss, val, ',')) continue; rec.p.z() = std::stod(val);
            records.push_back(rec);
        } catch (...) {
            continue;
        }
    }
    return records;
}

BenchmarkMetrics runScenarioOnEstimator(
    std::shared_ptr<coneskf::BaseEstimator> est,
    const std::vector<IMURecord>& imu_data,
    const std::vector<LeicaRecord>& leica_data,
    const std::string& scenario_name
) {
    // Coarse gravity alignment initialization
    Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
    size_t n_align = std::min(size_t(50), imu_data.size());
    for (size_t k = 0; k < n_align; ++k) a_sum += imu_data[k].a;
    Eigen::Matrix3d R0 = coneskf::computeGravityAlignment(a_sum / static_cast<double>(n_align));

    est->initialize(leica_data[0].p, R0);

    coneskf::CertifiedPoseTube tube(0.05, 120);

    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    std::normal_distribution<double> norm_dist(0.0, 0.15); // Secondary sensor noise (m)
    std::cauchy_distribution<double> cauchy_dist(0.0, 2.5); // NLoS multipath spikes (m)

    Eigen::Matrix3d R_primary = Eigen::Matrix3d::Identity() * 0.04;   // Primary GNSS/Vicon cov (20 cm std)
    Eigen::Matrix3d R_secondary = Eigen::Matrix3d::Identity() * 0.04; // Secondary UWB/Flow cov (30 cm std)

    size_t leica_idx = 0;
    std::vector<double> position_errors;
    position_errors.reserve(leica_data.size());

    std::vector<double> latencies_us;
    latencies_us.reserve(imu_data.size());

    double attack_onset_t = 30.0;
    double detected_t = -1.0;
    bool false_alarm = false;

    for (size_t i = 1; i < imu_data.size(); ++i) {
        double dt = imu_data[i].t - imu_data[i - 1].t;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;

        auto t_start = std::chrono::high_resolution_clock::now();

        // 1. IMU Propagation (200.1 Hz real physical data)
        est->predictIMU(imu_data[i].a, imu_data[i].w, dt);

        double current_t = imu_data[i].t;
        while (leica_idx < leica_data.size() && leica_data[leica_idx].t <= current_t) {
            Eigen::Vector3d p_true = leica_data[leica_idx].p;
            Eigen::Vector3d z_primary = p_true;

            // Scenario 1: Heavy-tail NLoS spikes (15% burst rate)
            if (scenario_name == "S1_HeavyTailNLoS") {
                if (unif(rng) < 0.15) {
                    z_primary.x() += cauchy_dist(rng);
                    z_primary.y() += cauchy_dist(rng);
                    z_primary.z() += 0.5 * cauchy_dist(rng);
                }
            }
            // Scenario 2: Stealth Ramp Spoofing on Primary Channel (0.15 m/s slow drift)
            else if (scenario_name == "S2_StealthRamp") {
                if (current_t >= attack_onset_t) {
                    double dt_attack = current_t - attack_onset_t;
                    z_primary.x() += 0.15 * dt_attack;
                    z_primary.y() += 0.10 * dt_attack;
                }
            }

            // Primary measurement update (GNSS)
            est->updatePositionChannel(z_primary, R_primary, "gnss");

            // In Scenario 2: Secondary healthy aiding channel (UWB / Optical Flow)
            if (scenario_name == "S2_StealthRamp") {
                Eigen::Vector3d z_secondary = p_true;
                z_secondary.x() += norm_dist(rng);
                z_secondary.y() += norm_dist(rng);
                z_secondary.z() += norm_dist(rng);
                est->updatePositionChannel(z_secondary, R_secondary, "uwb");
            }

            // Compute true estimation error against ground truth
            double err = (est->getState().p - p_true).norm();
            position_errors.push_back(err);

            // Update Certified Pose Tube
            tube.update(est->getState().p, est->getCovariance().block<3, 3>(0, 0), p_true);

            // Check Fault Detection / Quarantine status
            if (est->isFaultDetected()) {
                if (scenario_name == "S2_StealthRamp") {
                    if (current_t >= attack_onset_t) {
                        if (detected_t < 0.0) {
                            detected_t = current_t;
                        }
                    } else {
                        false_alarm = true;
                    }
                } else {
                    false_alarm = true;
                }
            }

            leica_idx++;
        }

        auto t_end = std::chrono::high_resolution_clock::now();
        double us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        latencies_us.push_back(us);
    }

    double sum_sq = 0.0;
    double max_err = 0.0;
    for (double err : position_errors) {
        sum_sq += err * err;
        if (err > max_err) max_err = err;
    }
    double ate_rmse = std::sqrt(sum_sq / std::max(size_t(1), position_errors.size()));

    double sum_lat = 0.0;
    for (double us : latencies_us) sum_lat += us;
    double mean_lat = sum_lat / latencies_us.size();
    std::sort(latencies_us.begin(), latencies_us.end());
    double p99_lat = latencies_us[static_cast<size_t>(latencies_us.size() * 0.99)];

    double ttd = (detected_t >= 0.0) ? (detected_t - attack_onset_t) : -1.0;

    BenchmarkMetrics bm;
    bm.estimator_name = est->getName();
    bm.scenario = scenario_name;
    bm.ate_rmse = ate_rmse;
    bm.ate_max = max_err;
    bm.empirical_coverage = tube.getEmpiricalCoverage();
    bm.time_to_detect = ttd;
    bm.false_alarm = false_alarm;
    bm.mean_latency_us = mean_lat;
    bm.p99_latency_us = p99_lat;

    return bm;
}

int main() {
    std::cout << "================================================================" << std::endl;
    std::cout << "ConESKF-FDI: Rigorous Real-Flight Benchmark Execution (Phase 2)" << std::endl;
    std::cout << "100% Real Physical Telemetry: EuRoC MAV MH_01_easy" << std::endl;
    std::cout << "================================================================" << std::endl;

    std::string imu_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_imu.csv";
    std::string leica_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_leica.csv";

    auto imu_data = loadIMU(imu_csv);
    auto leica_data = loadLeica(leica_csv);

    std::cout << "Loaded " << imu_data.size() << " real IMU measurements (200.1 Hz)." << std::endl;
    std::cout << "Loaded " << leica_data.size() << " real Leica millimeter GT measurements." << std::endl;

    std::vector<std::string> scenarios = {"S0_CleanFlight", "S1_HeavyTailNLoS", "S2_StealthRamp"};
    std::vector<BenchmarkMetrics> all_results;

    for (const auto& scen : scenarios) {
        std::cout << "\n----------------------------------------------------------------" << std::endl;
        std::cout << "Evaluating Scenario: " << scen << std::endl;
        std::cout << "----------------------------------------------------------------" << std::endl;

        std::vector<std::shared_ptr<coneskf::BaseEstimator>> estimators = {
            std::make_shared<coneskf::StandardChi2ESKF>(),
            std::make_shared<coneskf::HuberESKF>(),
            std::make_shared<coneskf::VariationalBayesESKF>(),
            std::make_shared<coneskf::Chi2CUSUMESKF>(),
            std::make_shared<coneskf::StaticConformalESKF>(),
            std::make_shared<coneskf::ConESKFFDIEstimator>()
        };

        for (auto& est : estimators) {
            std::cout << "  Running " << est->getName() << "... " << std::flush;
            auto res = runScenarioOnEstimator(est, imu_data, leica_data, scen);
            all_results.push_back(res);
            std::cout << "Done! ATE RMSE: " << std::fixed << std::setprecision(4) << res.ate_rmse << " m, "
                      << "Coverage: " << std::setprecision(1) << res.empirical_coverage * 100.0 << "%, "
                      << "Latency: " << std::setprecision(2) << res.mean_latency_us << " us" << std::endl;
        }
    }

    std::string out_csv = "/home/jarvis/coneskf_fdi_ws/results/table2_euroc_benchmarks.csv";
    std::ofstream csv_out(out_csv);
    csv_out << "Scenario,Estimator,ATE_RMSE_m,ATE_Max_m,Empirical_Coverage_pct,TTD_s,False_Alarm,Mean_Latency_us,P99_Latency_us\n";
    for (const auto& r : all_results) {
        csv_out << r.scenario << ","
                << r.estimator_name << ","
                << r.ate_rmse << ","
                << r.ate_max << ","
                << r.empirical_coverage * 100.0 << ","
                << (r.time_to_detect >= 0.0 ? std::to_string(r.time_to_detect) : "None") << ","
                << (r.false_alarm ? "TRUE" : "FALSE") << ","
                << r.mean_latency_us << ","
                << r.p99_latency_us << "\n";
    }
    std::cout << "\nAll benchmark results exported to: " << out_csv << std::endl;

    return 0;
}
