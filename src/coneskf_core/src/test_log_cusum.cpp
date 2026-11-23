
#include "coneskf_core/coneskf.hpp"
#include <iostream>
#include <fstream>
#include <vector>

void testLogCUSUM(double eps, double drift, double h_thresh) {
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

    // Run across all 3 scenarios
    auto run = [&](const std::string& scen) {
        coneskf::State15 state;
        state.p = leica_p[0].second; state.R = R0; state.v.setZero();
        Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;

        Eigen::Matrix3d R_gnss = Eigen::Matrix3d::Identity() * 0.04;
        Eigen::Matrix3d R_uwb = Eigen::Matrix3d::Identity() * 0.09;

        std::mt19937_64 rng(42);
        std::uniform_real_distribution<double> unif(0.0, 1.0);
        std::normal_distribution<double> norm_dist(0.0, 0.15);
        std::cauchy_distribution<double> cauchy_dist(0.0, 2.5);

        double attack_onset_t = 60.0;
        double detect_t = -1.0;
        double max_g = 0.0;
        double g = 0.0;
        bool quarantined = false;
        std::deque<double> history;
        std::vector<double> errors;
        size_t l_idx = 0;

        for (size_t i = 1; i < imu_a.size(); ++i) {
            double dt = imu_a[i].first - imu_a[i-1].first;
            if (dt <= 0.0 || dt > 0.1) dt = 0.005;

            // IMU Predict
            Eigen::Vector3d acc = imu_a[i].second - state.ba;
            Eigen::Vector3d gyr = imu_w[i].second - state.bg;
            Eigen::Matrix3d dR = coneskf::expSO3(gyr * dt);
            Eigen::Vector3d acc_w = state.R * acc + Eigen::Vector3d(0, 0, -9.80665);
            state.p += state.v * dt + 0.5 * acc_w * dt * dt;
            state.v += acc_w * dt;
            state.R = state.R * dR;

            Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
            F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
            F.block<3, 3>(0, 6) = -0.5 * state.R * coneskf::skew(acc) * (dt * dt);
            F.block<3, 3>(0, 9) = -0.5 * state.R * (dt * dt);
            F.block<3, 3>(3, 6) = -state.R * coneskf::skew(acc) * dt;
            F.block<3, 3>(3, 9) = -state.R * dt;
            F.block<3, 3>(6, 6) = coneskf::expSO3(-gyr * dt);
            F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

            Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
            Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.08 * 0.08 * dt * dt * dt / 3.0);
            Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (0.08 * 0.08 * dt);
            Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (0.004 * 0.004 * dt);
            Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (0.0004 * 0.0004 * dt);
            Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (0.00003 * 0.00003 * dt);
            P = F * P * F.transpose() + Qd;
            P = 0.5 * (P + P.transpose());

            double t = imu_a[i].first;
            while (l_idx < leica_p.size() && leica_p[l_idx].first <= t) {
                Eigen::Vector3d p_true = leica_p[l_idx].second;
                Eigen::Vector3d z_gnss = p_true;

                if (scen == "S1_HeavyTailNLoS") {
                    if (unif(rng) < 0.15) {
                        z_gnss.x() += cauchy_dist(rng);
                        z_gnss.y() += cauchy_dist(rng);
                        z_gnss.z() += 0.5 * cauchy_dist(rng);
                    }
                } else if (scen == "S2_StealthRamp") {
                    if (t >= attack_onset_t) {
                        double dt_att = t - attack_onset_t;
                        z_gnss.x() += 0.15 * dt_att;
                        z_gnss.y() += 0.10 * dt_att;
                    }
                }

                Eigen::Vector3d nu = z_gnss - state.p;
                Eigen::Matrix3d S = P.block<3, 3>(0, 0) + R_gnss;
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

                    double e_k = 1.0 + eps * (1.0 - 2.0 * p_k);
                    double score = std::log(e_k) - drift;
                    g = std::max(0.0, g + score);

                    if (g > max_g) max_g = g;
                    if (g >= h_thresh && !quarantined) {
                        quarantined = true;
                        detect_t = t;
                    }
                }

                if (!quarantined) {
                    history.push_back(s_k);
                    if (history.size() > 200) history.pop_front();
                }

                // GNSS update if healthy
                if (!quarantined) {
                    Eigen::Matrix<double, 15, 3> H_t = Eigen::Matrix<double, 15, 3>::Zero();
                    H_t.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                    Eigen::Matrix<double, 15, 3> K = P * H_t;
                    K = llt.solve(K.transpose()).transpose();
                    Eigen::Matrix<double, 15, 1> dx = K * nu;
                    Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity();
                    I_KH.block<15, 3>(0, 0) -= K;
                    P = I_KH * P * I_KH.transpose() + K * R_gnss * K.transpose();
                    P = 0.5 * (P + P.transpose());
                    state.p += dx.segment<3>(0);
                    state.v += dx.segment<3>(3);
                    state.R = state.R * coneskf::expSO3(dx.segment<3>(6));
                    state.ba += dx.segment<3>(9);
                    state.bg += dx.segment<3>(12);
                }

                // UWB update
                if (scen == "S2_StealthRamp") {
                    Eigen::Vector3d z_uwb = p_true;
                    z_uwb.x() += norm_dist(rng);
                    z_uwb.y() += norm_dist(rng);
                    z_uwb.z() += norm_dist(rng);

                    Eigen::Vector3d nu_u = z_uwb - state.p;
                    Eigen::Matrix3d S_u = P.block<3, 3>(0, 0) + R_uwb;
                    Eigen::LLT<Eigen::Matrix3d> llt_u(S_u);
                    Eigen::Matrix<double, 15, 3> H_t = Eigen::Matrix<double, 15, 3>::Zero();
                    H_t.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                    Eigen::Matrix<double, 15, 3> K_u = P * H_t;
                    K_u = llt_u.solve(K_u.transpose()).transpose();
                    Eigen::Matrix<double, 15, 1> dx_u = K_u * nu_u;
                    Eigen::Matrix<double, 15, 15> I_KH_u = Eigen::Matrix<double, 15, 15>::Identity();
                    I_KH_u.block<15, 3>(0, 0) -= K_u;
                    P = I_KH_u * P * I_KH_u.transpose() + K_u * R_uwb * K_u.transpose();
                    P = 0.5 * (P + P.transpose());
                    state.p += dx_u.segment<3>(0);
                    state.v += dx_u.segment<3>(3);
                    state.R = state.R * coneskf::expSO3(dx_u.segment<3>(6));
                    state.ba += dx_u.segment<3>(9);
                    state.bg += dx_u.segment<3>(12);
                }

                errors.push_back((state.p - p_true).norm());
                l_idx++;
            }
        }

        double sum_sq = 0, max_e = 0;
        for (double e : errors) {
            sum_sq += e * e;
            if (e > max_e) max_e = e;
        }
        double rmse = std::sqrt(sum_sq / errors.size());
        double ttd = (detect_t >= 0.0) ? (detect_t - attack_onset_t) : -1.0;
        bool fa = (detect_t >= 0.0 && detect_t < attack_onset_t);

        std::cout << "  [" << scen << "] RMSE=" << rmse << " m, Max=" << max_e 
                  << " m, Max_g=" << max_g << " (thresh=" << h_thresh << "), TTD=" 
                  << (ttd >= 0 ? std::to_string(ttd) + "s" : "None") 
                  << ", FalseAlarm=" << (fa ? "TRUE" : "FALSE") << std::endl;
    };

    std::cout << "Config [eps=" << eps << ", drift=" << drift << ", h=" << h_thresh << "]:\n";
    run("S0_CleanFlight");
    run("S1_HeavyTailNLoS");
    run("S2_StealthRamp");
}

int main() {
    testLogCUSUM(0.20, 0.02, 4.605); // ln(100) = 4.605
    testLogCUSUM(0.25, 0.025, 4.605);
    testLogCUSUM(0.30, 0.03, 4.605);
    return 0;
}
