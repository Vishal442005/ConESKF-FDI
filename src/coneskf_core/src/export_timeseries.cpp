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

int main() {
    std::string imu_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_imu.csv";
    std::string leica_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_leica.csv";

    auto imu_data = loadIMU(imu_csv);
    auto leica_data = loadLeica(leica_csv);

    // Initial gravity alignment
    Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();
    size_t n_align = std::min(size_t(50), imu_data.size());
    for (size_t k = 0; k < n_align; ++k) a_sum += imu_data[k].a;
    Eigen::Matrix3d R0 = coneskf::computeGravityAlignment(a_sum / static_cast<double>(n_align));

    // -------------------------------------------------------------
    // EXPORT 1: Scenario 2 Stealth Ramp Time-Series
    // -------------------------------------------------------------
    std::cout << "Simulating Scenario 2 (Stealth Ramp) with detailed logging..." << std::endl;
    auto b1 = std::make_shared<coneskf::StandardChi2ESKF>();
    auto b4 = std::make_shared<coneskf::Chi2CUSUMESKF>();
    auto b6 = std::make_shared<coneskf::ConESKFFDIEstimator>();

    b1->initialize(leica_data[0].p, R0);
    b4->initialize(leica_data[0].p, R0);
    b6->initialize(leica_data[0].p, R0);

    coneskf::CertifiedPoseTube tube(0.05, 120);

    std::mt19937_64 rng(42);
    std::normal_distribution<double> norm_dist(0.0, 0.15);
    Eigen::Matrix3d R_primary = Eigen::Matrix3d::Identity() * 0.04;
    Eigen::Matrix3d R_secondary = Eigen::Matrix3d::Identity() * 0.04;

    double attack_onset_t = 30.0;
    size_t leica_idx = 0;

    std::ofstream f_s2("/home/jarvis/coneskf_fdi_ws/results/timeseries_s2.csv");
    f_s2 << "t,gt_x,gt_y,gt_z,spoof_drift_x,b1_x,b1_y,b1_z,b1_err,b4_x,b4_y,b4_z,b4_err,b4_cusum,b6_x,b6_y,b6_z,b6_err,b6_wealth,b6_radius,b6_in_tube,chi2_stat\n";

    for (size_t i = 1; i < imu_data.size(); ++i) {
        double dt = imu_data[i].t - imu_data[i - 1].t;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;

        b1->predictIMU(imu_data[i].a, imu_data[i].w, dt);
        b4->predictIMU(imu_data[i].a, imu_data[i].w, dt);
        b6->predictIMU(imu_data[i].a, imu_data[i].w, dt);

        double current_t = imu_data[i].t;
        while (leica_idx < leica_data.size() && leica_data[leica_idx].t <= current_t) {
            Eigen::Vector3d p_true = leica_data[leica_idx].p;
            Eigen::Vector3d z_primary = p_true;
            double drift_x = 0.0;

            if (current_t >= attack_onset_t) {
                double dt_attack = current_t - attack_onset_t;
                drift_x = 0.15 * dt_attack;
                z_primary.x() += drift_x;
                z_primary.y() += 0.10 * dt_attack;
            }

            // Update primary
            b1->updatePositionChannel(z_primary, R_primary, "gnss");
            b4->updatePositionChannel(z_primary, R_primary, "gnss");
            b6->updatePositionChannel(z_primary, R_primary, "gnss");

            // Secondary healthy channel
            Eigen::Vector3d z_secondary = p_true;
            z_secondary.x() += norm_dist(rng);
            z_secondary.y() += norm_dist(rng);
            z_secondary.z() += norm_dist(rng);
            b1->updatePositionChannel(z_secondary, R_secondary, "uwb");
            b4->updatePositionChannel(z_secondary, R_secondary, "uwb");
            b6->updatePositionChannel(z_secondary, R_secondary, "uwb");

            double b1_err = (b1->getState().p - p_true).norm();
            double b4_err = (b4->getState().p - p_true).norm();
            double b6_err = (b6->getState().p - p_true).norm();

            double rad = 0.0;
            bool covered = tube.update(b6->getState().p, b6->getCovariance().block<3, 3>(0, 0), p_true, &rad);

            double w = b6->getMartingaleWealth("gnss");
            double cusum_val = b4->getCusumScore();

            // Compute standard chi2 innovation statistic on primary channel
            Eigen::Vector3d nu_primary = z_primary - b1->getState().p;
            Eigen::Matrix3d S_primary = b1->getCovariance().block<3, 3>(0, 0) + R_primary;
            double chi2_val = nu_primary.transpose() * S_primary.inverse() * nu_primary;

            f_s2 << std::fixed << std::setprecision(5)
                 << current_t << ","
                 << p_true.x() << "," << p_true.y() << "," << p_true.z() << ","
                 << drift_x << ","
                 << b1->getState().p.x() << "," << b1->getState().p.y() << "," << b1->getState().p.z() << ","
                 << b1_err << ","
                 << b4->getState().p.x() << "," << b4->getState().p.y() << "," << b4->getState().p.z() << ","
                 << b4_err << ","
                 << cusum_val << ","
                 << b6->getState().p.x() << "," << b6->getState().p.y() << "," << b6->getState().p.z() << ","
                 << b6_err << ","
                 << w << ","
                 << rad << ","
                 << (covered ? 1 : 0) << ","
                 << chi2_val << "\n";

            leica_idx++;
        }
    }
    f_s2.close();
    std::cout << "timeseries_s2.csv written successfully!" << std::endl;

    // -------------------------------------------------------------
    // EXPORT 2: Scenario 0 Clean Flight Time-Series
    // -------------------------------------------------------------
    std::cout << "Simulating Scenario 0 (Clean Flight) with detailed logging..." << std::endl;
    auto b1_s0 = std::make_shared<coneskf::StandardChi2ESKF>();
    auto b6_s0 = std::make_shared<coneskf::ConESKFFDIEstimator>();
    b1_s0->initialize(leica_data[0].p, R0);
    b6_s0->initialize(leica_data[0].p, R0);
    coneskf::CertifiedPoseTube tube_s0(0.05, 120);

    leica_idx = 0;
    std::ofstream f_s0("/home/jarvis/coneskf_fdi_ws/results/timeseries_s0.csv");
    f_s0 << "t,gt_x,gt_y,gt_z,b1_x,b1_y,b1_z,b1_err,b6_x,b6_y,b6_z,b6_err,b6_wealth,b6_radius,b6_in_tube\n";

    for (size_t i = 1; i < imu_data.size(); ++i) {
        double dt = imu_data[i].t - imu_data[i - 1].t;
        if (dt <= 0.0 || dt > 0.1) dt = 0.005;

        b1_s0->predictIMU(imu_data[i].a, imu_data[i].w, dt);
        b6_s0->predictIMU(imu_data[i].a, imu_data[i].w, dt);

        double current_t = imu_data[i].t;
        while (leica_idx < leica_data.size() && leica_data[leica_idx].t <= current_t) {
            Eigen::Vector3d p_true = leica_data[leica_idx].p;
            b1_s0->updatePositionChannel(p_true, R_primary, "gnss");
            b6_s0->updatePositionChannel(p_true, R_primary, "gnss");

            double b1_err = (b1_s0->getState().p - p_true).norm();
            double b6_err = (b6_s0->getState().p - p_true).norm();
            double rad = 0.0;
            bool covered = tube_s0.update(b6_s0->getState().p, b6_s0->getCovariance().block<3, 3>(0, 0), p_true, &rad);
            double w = b6_s0->getMartingaleWealth("gnss");

            f_s0 << std::fixed << std::setprecision(5)
                 << current_t << ","
                 << p_true.x() << "," << p_true.y() << "," << p_true.z() << ","
                 << b1_s0->getState().p.x() << "," << b1_s0->getState().p.y() << "," << b1_s0->getState().p.z() << ","
                 << b1_err << ","
                 << b6_s0->getState().p.x() << "," << b6_s0->getState().p.y() << "," << b6_s0->getState().p.z() << ","
                 << b6_err << ","
                 << w << ","
                 << rad << ","
                 << (covered ? 1 : 0) << "\n";

            leica_idx++;
        }
    }
    f_s0.close();
    std::cout << "timeseries_s0.csv written successfully!" << std::endl;

    // -------------------------------------------------------------
    // EXPORT 3: Calibration Curve (Nominal vs Empirical Coverage)
    // -------------------------------------------------------------
    std::cout << "Evaluating Calibration Curves across confidence levels..." << std::endl;
    std::ofstream f_cal("/home/jarvis/coneskf_fdi_ws/results/calibration_curve.csv");
    f_cal << "nominal_confidence,nominal_alpha,conformal_empirical_coverage,gaussian_empirical_coverage\n";

    for (int conf_pct = 80; conf_pct <= 99; conf_pct++) {
        double nominal_conf = conf_pct / 100.0;
        double alpha = 1.0 - nominal_conf;

        auto est = std::make_shared<coneskf::ConESKFFDIEstimator>();
        est->initialize(leica_data[0].p, R0);
        coneskf::CertifiedPoseTube cal_tube(alpha, 120);

        size_t l_idx = 0;
        size_t g_covered = 0;
        size_t total = 0;

        // chi2 3-dof inverse cdf quantile for Gaussian baseline
        // For 3 dof: 80% -> 4.64, 90% -> 6.25, 95% -> 7.81, 99% -> 11.34
        // Approximation: k * (1 - 2/(9k) + z * sqrt(2/(9k)))^3 with Wilson-Hilferty
        double z = 0.0;
        if (conf_pct == 80) z = 0.8416;
        else if (conf_pct == 85) z = 1.0364;
        else if (conf_pct == 90) z = 1.2816;
        else if (conf_pct == 95) z = 1.6449;
        else if (conf_pct == 98) z = 2.0537;
        else if (conf_pct == 99) z = 2.3263;
        else z = 1.2816 + (conf_pct - 90) * 0.11; // linear interp

        double wh = 1.0 - 2.0/27.0 + z * std::sqrt(2.0/27.0);
        double chi2_crit = 3.0 * wh * wh * wh;

        for (size_t i = 1; i < imu_data.size(); ++i) {
            double dt = imu_data[i].t - imu_data[i - 1].t;
            if (dt <= 0.0 || dt > 0.1) dt = 0.005;
            est->predictIMU(imu_data[i].a, imu_data[i].w, dt);

            double current_t = imu_data[i].t;
            while (l_idx < leica_data.size() && leica_data[l_idx].t <= current_t) {
                Eigen::Vector3d p_true = leica_data[l_idx].p;
                est->updatePositionChannel(p_true, R_primary, "gnss");

                cal_tube.update(est->getState().p, est->getCovariance().block<3, 3>(0, 0), p_true);

                // Gaussian coverage check: (p - p_true)^T P^{-1} (p - p_true) <= chi2_crit
                Eigen::Vector3d err = p_true - est->getState().p;
                Eigen::Matrix3d Ppos = est->getCovariance().block<3, 3>(0, 0);
                double maha = err.transpose() * Ppos.inverse() * err;
                if (maha <= chi2_crit) g_covered++;
                total++;

                l_idx++;
            }
        }

        double emp_conf = cal_tube.getEmpiricalCoverage();
        double g_conf = (total > 0) ? (static_cast<double>(g_covered) / total) : 0.0;

        f_cal << std::fixed << std::setprecision(4)
              << nominal_conf << "," << alpha << ","
              << emp_conf << "," << g_conf << "\n";
        std::cout << "  Conf: " << conf_pct << "% -> Conformal: " << emp_conf * 100.0
                  << "%, Gaussian: " << g_conf * 100.0 << "%" << std::endl;
    }
    f_cal.close();
    std::cout << "calibration_curve.csv written successfully!" << std::endl;

    return 0;
}
