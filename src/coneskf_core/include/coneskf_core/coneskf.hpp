#pragma once

#include "so3_math.hpp"
#include "conformal_martingale.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <deque>
#include <iostream>

namespace coneskf {

struct State15 {
    Eigen::Vector3d p;    // 3D position (m)
    Eigen::Vector3d v;    // 3D velocity (m/s)
    Eigen::Matrix3d R;    // SO(3) attitude rotation matrix
    Eigen::Vector3d ba;   // Accelerometer bias (m/s^2)
    Eigen::Vector3d bg;   // Gyroscope bias (rad/s)

    State15() {
        p.setZero();
        v.setZero();
        R.setIdentity();
        ba.setZero();
        bg.setZero();
    }
};

struct ACIChannel {
    double alpha_target;
    double alpha_current;
    double gamma_lr;
    double alpha_min;
    double alpha_max;
    double kappa_power;
    double current_quantile;
    double current_inflation;
    std::deque<double> norm_history;
    size_t window_size;

    ACIChannel(
        double target = 0.05,
        double gamma = 0.02,
        double kappa = 2.0,
        size_t win = 200
    ) : alpha_target(target),
        alpha_current(target),
        gamma_lr(gamma),
        alpha_min(0.001),
        alpha_max(0.30),
        kappa_power(kappa),
        current_quantile(3.0),
        current_inflation(1.0),
        window_size(win) {}

    // Evaluates empirical quantile and updates miscoverage online
    double update(double s_k) {
        norm_history.push_back(s_k);
        if (norm_history.size() > window_size) {
            norm_history.pop_front();
        }

        // Empirical quantile of norm_history at 1 - alpha_current
        std::vector<double> sorted_norms(norm_history.begin(), norm_history.end());
        std::sort(sorted_norms.begin(), sorted_norms.end());
        
        size_t q_idx = static_cast<size_t>(std::ceil((1.0 - alpha_current) * sorted_norms.size()));
        q_idx = std::clamp(q_idx, static_cast<size_t>(0), sorted_norms.size() - 1);
        current_quantile = sorted_norms[q_idx];

        // Miscoverage error: err_k = I[s_k > q_k]
        double err_k = (s_k > current_quantile) ? 1.0 : 0.0;

        // Online gradient step on alpha
        alpha_current += gamma_lr * (alpha_target - err_k);
        alpha_current = std::clamp(alpha_current, alpha_min, alpha_max);

        // Smooth redescending power inflation: phi = (s / q)^kappa if s > q else 1.0
        if (s_k > current_quantile && current_quantile > 1e-6) {
            current_inflation = std::pow(s_k / current_quantile, kappa_power);
        } else {
            current_inflation = 1.0;
        }

        return current_inflation;
    }
};

class ConESKFFilter {
public:
    ConESKFFilter() {
        gravity_ = Eigen::Vector3d(0.0, 0.0, -9.80665);
        state_ = State15();
        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 0.1;   // Position cov
        P_.block<3, 3>(3, 3) *= 0.1;   // Velocity cov
        P_.block<3, 3>(6, 6) *= 0.01;  // Rotation cov
        P_.block<3, 3>(9, 9) *= 0.001; // Accel bias cov
        P_.block<3, 3>(12, 12) *= 0.0001; // Gyro bias cov

        // Continuous IMU noise spectral densities
        acc_noise_ = 0.08;       // m/s^2 / sqrt(Hz)
        gyr_noise_ = 0.004;      // rad/s / sqrt(Hz)
        acc_bias_noise_ = 0.0004; // m/s^3 / sqrt(Hz)
        gyr_bias_noise_ = 0.00003; // rad/s^2 / sqrt(Hz)
    }

    // Set initial position and orientation
    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) {
        state_.p = p0;
        state_.R = R0;
        state_.v.setZero();
    }

    // 15-State SO(3) Kinematic Propagation at IMU sampling period dt
    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& omega_m, double dt) {
        // Debias IMU readings
        Eigen::Vector3d acc = a_m - state_.ba;
        Eigen::Vector3d gyr = omega_m - state_.bg;

        // 1. Nominal State Kinematics
        Eigen::Matrix3d dR = expSO3(gyr * dt);
        Eigen::Vector3d acc_world = state_.R * acc + gravity_;

        state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
        state_.v += acc_world * dt;
        state_.R = state_.R * dR;

        // 2. Error State Transition Jacobian F_k (15x15)
        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * state_.R * skew(acc) * (dt * dt);
        F.block<3, 3>(0, 9) = -0.5 * state_.R * (dt * dt);

        F.block<3, 3>(3, 6) = -state_.R * skew(acc) * dt;
        F.block<3, 3>(3, 9) = -state_.R * dt;

        F.block<3, 3>(6, 6) = expSO3(-gyr * dt);
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        // 3. Discrete Process Noise Covariance Q_d (15x15)
        Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
        Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt * dt * dt / 3.0);
        Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt);
        Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (gyr_noise_ * gyr_noise_ * dt);
        Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (acc_bias_noise_ * acc_bias_noise_ * dt);
        Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (gyr_bias_noise_ * gyr_bias_noise_ * dt);

        // Covariance Propagation: P_{k|k-1} = F * P_{k-1} * F^T + Q_d
        P_ = F * P_ * F.transpose() + Qd;
        // Enforce exact symmetry
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // Aided 3D Position Measurement Update (GNSS / UWB / Vicon)
    bool updatePosition(
        const Eigen::Vector3d& z_pos,
        const Eigen::Matrix3d& R_cov,
        const std::string& modality_id,
        double* out_whitened_norm = nullptr,
        double* out_inflation = nullptr,
        double* out_martingale = nullptr
    ) {
        // Initialize channel objects if not present
        if (aci_channels_.find(modality_id) == aci_channels_.end()) {
            aci_channels_[modality_id] = ACIChannel();
            martingales_[modality_id] = ConformalBettingMartingale();
        }

        auto& aci = aci_channels_[modality_id];
        auto& mart = martingales_[modality_id];

        // 1. Innovation Residual: nu = z - p_hat
        Eigen::Vector3d nu = z_pos - state_.p;

        // 2. Innovation Covariance: S = H * P * H^T + R
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S = H * P_ * H.transpose() + R_cov;

        // Cholesky Factorization: S = L * L^T
        Eigen::LLT<Eigen::Matrix3d> lltOfS(S);
        if (lltOfS.info() != Eigen::Success) {
            std::cerr << "[ConESKF] Cholesky factorization failed on S! Matrix not PD." << std::endl;
            return false;
        }

        // 3. Whitened Innovation Norm: s_k = ||L^{-1} * nu||_2
        Eigen::Vector3d whitened_nu = lltOfS.matrixL().solve(nu);
        double s_k = whitened_nu.norm();

        if (out_whitened_norm) *out_whitened_norm = s_k;

        // 4. Online Betting Martingale Step
        double m_wealth = mart.update(s_k);
        if (out_martingale) *out_martingale = m_wealth;

        // Quarantine check: If martingale rejected this channel, isolate and skip measurement
        if (mart.isQuarantined()) {
            return false;
        }

        // 5. Online ACI Covariance Inflation factor phi_k
        double phi_k = aci.update(s_k);
        if (out_inflation) *out_inflation = phi_k;

        // Inflated measurement covariance: R_tilde = phi_k * R_cov
        Eigen::Matrix3d R_tilde = phi_k * R_cov;

        // Inflated S_tilde = H * P * H^T + R_tilde
        Eigen::Matrix3d S_tilde = H * P_ * H.transpose() + R_tilde;
        Eigen::LLT<Eigen::Matrix3d> lltInflated(S_tilde);
        if (lltInflated.info() != Eigen::Success) {
            return false;
        }

        // 6. Kalman Gain: K = P * H^T * (S_tilde)^{-1}
        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
        K = lltInflated.solve(K.transpose()).transpose();

        // 7. Error State Correction: delta_x = K * nu
        Eigen::Matrix<double, 15, 1> delta_x = K * nu;

        // 8. Joseph-form Stabilized Covariance Update:
        // P = (I - K*H) * P * (I - K*H)^T + K * R_tilde * K^T
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_tilde * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        // 9. Manifold Composition Injection into Nominal State
        state_.p += delta_x.segment<3>(0);
        state_.v += delta_x.segment<3>(3);
        state_.R = state_.R * expSO3(delta_x.segment<3>(6));
        state_.ba += delta_x.segment<3>(9);
        state_.bg += delta_x.segment<3>(12);

        return true;
    }

    const State15& getState() const { return state_; }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const { return P_; }

    bool isChannelQuarantined(const std::string& modality_id) {
        if (martingales_.find(modality_id) != martingales_.end()) {
            return martingales_[modality_id].isQuarantined();
        }
        return false;
    }

    double getMartingaleWealth(const std::string& modality_id) const {
        auto it = martingales_.find(modality_id);
        if (it != martingales_.end()) {
            return it->second.getWealth();
        }
        return 1.0;
    }

private:
    State15 state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Vector3d gravity_;

    double acc_noise_;
    double gyr_noise_;
    double acc_bias_noise_;
    double gyr_bias_noise_;

    std::map<std::string, ACIChannel> aci_channels_;
    std::map<std::string, ConformalBettingMartingale> martingales_;
};

} // namespace coneskf
