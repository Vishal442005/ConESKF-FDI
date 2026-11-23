#pragma once

#include "so3_math.hpp"
#include "coneskf.hpp"
#include "conformal_martingale.hpp"
#include "pose_tube.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>

namespace coneskf {

// Base interface for all 6 comparative state estimators
class BaseEstimator {
public:
    virtual ~BaseEstimator() = default;
    virtual void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) = 0;
    virtual void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) = 0;
    virtual bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov) = 0;
    virtual bool updatePositionChannel(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov, const std::string& /*modality_id*/) {
        return updatePosition(z_pos, R_cov);
    }
    virtual const State15& getState() const = 0;
    virtual const Eigen::Matrix<double, 15, 15>& getCovariance() const = 0;
    virtual std::string getName() const = 0;
    virtual bool isFaultDetected() const { return false; }
};

// =========================================================================
// Baseline 1: Standard SO(3) ESKF with 99% Chi^2 Mahalanobis Gating (PX4 EKF2)
// =========================================================================
class StandardChi2ESKF : public BaseEstimator {
public:
    StandardChi2ESKF(double chi2_thresh = 11.345) // 99% quantile for 3 DoF
        : chi2_thresh_(chi2_thresh), gravity_(0.0, 0.0, -9.80665) {
        state_ = State15();
        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 0.1;
        P_.block<3, 3>(3, 3) *= 0.1;
        P_.block<3, 3>(6, 6) *= 0.01;
        P_.block<3, 3>(9, 9) *= 0.001;
        P_.block<3, 3>(12, 12) *= 0.0001;

        acc_noise_ = 0.08;
        gyr_noise_ = 0.004;
        acc_bias_noise_ = 0.0004;
        gyr_bias_noise_ = 0.00003;
    }

    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) override {
        state_.p = p0;
        state_.R = R0;
        state_.v.setZero();
    }

    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) override {
        Eigen::Vector3d acc = a_m - state_.ba;
        Eigen::Vector3d gyr = w_m - state_.bg;

        Eigen::Matrix3d dR = expSO3(gyr * dt);
        Eigen::Vector3d acc_world = state_.R * acc + gravity_;

        state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
        state_.v += acc_world * dt;
        state_.R = state_.R * dR;

        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * state_.R * skew(acc) * (dt * dt);
        F.block<3, 3>(0, 9) = -0.5 * state_.R * (dt * dt);
        F.block<3, 3>(3, 6) = -state_.R * skew(acc) * dt;
        F.block<3, 3>(3, 9) = -state_.R * dt;
        F.block<3, 3>(6, 6) = expSO3(-gyr * dt);
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
        Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt * dt * dt / 3.0);
        Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt);
        Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (gyr_noise_ * gyr_noise_ * dt);
        Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (acc_bias_noise_ * acc_bias_noise_ * dt);
        Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (gyr_bias_noise_ * gyr_bias_noise_ * dt);

        P_ = F * P_ * F.transpose() + Qd;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov) override {
        Eigen::Vector3d nu = z_pos - state_.p;
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S = H * P_ * H.transpose() + R_cov;
        Eigen::LLT<Eigen::Matrix3d> llt(S);
        if (llt.info() != Eigen::Success) return false;

        Eigen::Vector3d w_nu = llt.matrixL().solve(nu);
        double d2 = w_nu.squaredNorm();

        // Standard hard Chi^2 gating: if Mahalanobis distance > threshold, discard
        if (d2 > chi2_thresh_) {
            return false;
        }

        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
        K = llt.solve(K.transpose()).transpose();

        Eigen::Matrix<double, 15, 1> delta_x = K * nu;
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_cov * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        state_.p += delta_x.segment<3>(0);
        state_.v += delta_x.segment<3>(3);
        state_.R = state_.R * expSO3(delta_x.segment<3>(6));
        state_.ba += delta_x.segment<3>(9);
        state_.bg += delta_x.segment<3>(12);

        return true;
    }

    const State15& getState() const override { return state_; }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const override { return P_; }
    std::string getName() const override { return "B1_StandardChi2ESKF"; }

private:
    double chi2_thresh_;
    State15 state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Vector3d gravity_;
    double acc_noise_, gyr_noise_, acc_bias_noise_, gyr_bias_noise_;
};

// =========================================================================
// Baseline 2: Huber Robust M-Estimator ESKF
// =========================================================================
class HuberESKF : public BaseEstimator {
public:
    HuberESKF(double huber_c = 1.345)
        : huber_c_(huber_c), gravity_(0.0, 0.0, -9.80665) {
        state_ = State15();
        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 0.1;
        P_.block<3, 3>(3, 3) *= 0.1;
        P_.block<3, 3>(6, 6) *= 0.01;
        P_.block<3, 3>(9, 9) *= 0.001;
        P_.block<3, 3>(12, 12) *= 0.0001;

        acc_noise_ = 0.08;
        gyr_noise_ = 0.004;
        acc_bias_noise_ = 0.0004;
        gyr_bias_noise_ = 0.00003;
    }

    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) override {
        state_.p = p0;
        state_.R = R0;
        state_.v.setZero();
    }

    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) override {
        Eigen::Vector3d acc = a_m - state_.ba;
        Eigen::Vector3d gyr = w_m - state_.bg;

        Eigen::Matrix3d dR = expSO3(gyr * dt);
        Eigen::Vector3d acc_world = state_.R * acc + gravity_;

        state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
        state_.v += acc_world * dt;
        state_.R = state_.R * dR;

        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * state_.R * skew(acc) * (dt * dt);
        F.block<3, 3>(0, 9) = -0.5 * state_.R * (dt * dt);
        F.block<3, 3>(3, 6) = -state_.R * skew(acc) * dt;
        F.block<3, 3>(3, 9) = -state_.R * dt;
        F.block<3, 3>(6, 6) = expSO3(-gyr * dt);
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
        Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt * dt * dt / 3.0);
        Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt);
        Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (gyr_noise_ * gyr_noise_ * dt);
        Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (acc_bias_noise_ * acc_bias_noise_ * dt);
        Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (gyr_bias_noise_ * gyr_bias_noise_ * dt);

        P_ = F * P_ * F.transpose() + Qd;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov) override {
        Eigen::Vector3d nu = z_pos - state_.p;
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S = H * P_ * H.transpose() + R_cov;
        Eigen::LLT<Eigen::Matrix3d> llt(S);
        if (llt.info() != Eigen::Success) return false;

        Eigen::Vector3d w_nu = llt.matrixL().solve(nu);
        double norm_nu = w_nu.norm();

        // Huber M-estimator weight: w = 1.0 if norm <= c else c / norm
        double weight = 1.0;
        if (norm_nu > huber_c_) {
            weight = huber_c_ / norm_nu;
        }

        // Inflated covariance: R_huber = (1 / weight) * R_cov
        Eigen::Matrix3d R_huber = (1.0 / weight) * R_cov;
        Eigen::Matrix3d S_huber = H * P_ * H.transpose() + R_huber;
        Eigen::LLT<Eigen::Matrix3d> lltH(S_huber);
        if (lltH.info() != Eigen::Success) return false;

        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
        K = lltH.solve(K.transpose()).transpose();

        Eigen::Matrix<double, 15, 1> delta_x = K * nu;
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_huber * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        state_.p += delta_x.segment<3>(0);
        state_.v += delta_x.segment<3>(3);
        state_.R = state_.R * expSO3(delta_x.segment<3>(6));
        state_.ba += delta_x.segment<3>(9);
        state_.bg += delta_x.segment<3>(12);

        return true;
    }

    const State15& getState() const override { return state_; }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const override { return P_; }
    std::string getName() const override { return "B2_HuberESKF"; }

private:
    double huber_c_;
    State15 state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Vector3d gravity_;
    double acc_noise_, gyr_noise_, acc_bias_noise_, gyr_bias_noise_;
};

// =========================================================================
// Baseline 3: Variational Bayesian Student-t Adaptive KF (VB-ESKF)
// (Huang et al., IEEE Trans. Signal Processing 2022)
// =========================================================================
class VariationalBayesESKF : public BaseEstimator {
public:
    VariationalBayesESKF(int num_iterations = 4, double nu_dof = 4.0)
        : num_iterations_(num_iterations), nu_dof_(nu_dof), gravity_(0.0, 0.0, -9.80665) {
        state_ = State15();
        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 0.1;
        P_.block<3, 3>(3, 3) *= 0.1;
        P_.block<3, 3>(6, 6) *= 0.01;
        P_.block<3, 3>(9, 9) *= 0.001;
        P_.block<3, 3>(12, 12) *= 0.0001;

        acc_noise_ = 0.08;
        gyr_noise_ = 0.004;
        acc_bias_noise_ = 0.0004;
        gyr_bias_noise_ = 0.00003;
    }

    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) override {
        state_.p = p0;
        state_.R = R0;
        state_.v.setZero();
    }

    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) override {
        Eigen::Vector3d acc = a_m - state_.ba;
        Eigen::Vector3d gyr = w_m - state_.bg;

        Eigen::Matrix3d dR = expSO3(gyr * dt);
        Eigen::Vector3d acc_world = state_.R * acc + gravity_;

        state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
        state_.v += acc_world * dt;
        state_.R = state_.R * dR;

        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * state_.R * skew(acc) * (dt * dt);
        F.block<3, 3>(0, 9) = -0.5 * state_.R * (dt * dt);
        F.block<3, 3>(3, 6) = -state_.R * skew(acc) * dt;
        F.block<3, 3>(3, 9) = -state_.R * dt;
        F.block<3, 3>(6, 6) = expSO3(-gyr * dt);
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
        Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt * dt * dt / 3.0);
        Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt);
        Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (gyr_noise_ * gyr_noise_ * dt);
        Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (acc_bias_noise_ * acc_bias_noise_ * dt);
        Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (gyr_bias_noise_ * gyr_bias_noise_ * dt);

        P_ = F * P_ * F.transpose() + Qd;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov) override {
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d R_vb = R_cov;
        State15 state_iter = state_;
        Eigen::Matrix<double, 15, 15> P_iter = P_;

        // Iterative Variational Bayes updates
        for (int iter = 0; iter < num_iterations_; ++iter) {
            Eigen::Vector3d nu = z_pos - state_iter.p;
            Eigen::Matrix3d S = H * P_ * H.transpose() + R_vb;
            Eigen::LLT<Eigen::Matrix3d> llt(S);
            if (llt.info() != Eigen::Success) break;

            Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
            K = llt.solve(K.transpose()).transpose();

            Eigen::Matrix<double, 15, 1> delta_x = K * nu;
            Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
            P_iter = I_KH * P_ * I_KH.transpose() + K * R_vb * K.transpose();

            // Variational expectation of scale parameter:
            Eigen::Vector3d e_res = z_pos - (state_.p + delta_x.segment<3>(0));
            double E_quad = e_res.transpose() * R_cov.inverse() * e_res + (H * P_iter * H.transpose() * R_cov.inverse()).trace();
            double u_hat = (nu_dof_ + 3.0) / (nu_dof_ + E_quad);

            R_vb = (1.0 / std::max(1e-4, u_hat)) * R_cov;
        }

        // Final injection with adapted R_vb
        Eigen::Vector3d nu = z_pos - state_.p;
        Eigen::Matrix3d S = H * P_ * H.transpose() + R_vb;
        Eigen::LLT<Eigen::Matrix3d> llt(S);
        if (llt.info() != Eigen::Success) return false;

        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
        K = llt.solve(K.transpose()).transpose();

        Eigen::Matrix<double, 15, 1> delta_x = K * nu;
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_vb * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        state_.p += delta_x.segment<3>(0);
        state_.v += delta_x.segment<3>(3);
        state_.R = state_.R * expSO3(delta_x.segment<3>(6));
        state_.ba += delta_x.segment<3>(9);
        state_.bg += delta_x.segment<3>(12);

        return true;
    }

    const State15& getState() const override { return state_; }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const override { return P_; }
    std::string getName() const override { return "B3_VariationalBayesESKF"; }

private:
    int num_iterations_;
    double nu_dof_;
    State15 state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Vector3d gravity_;
    double acc_noise_, gyr_noise_, acc_bias_noise_, gyr_bias_noise_;
};

// =========================================================================
// Baseline 4: Standard ESKF + Chi^2 CUSUM Detector
// =========================================================================
class Chi2CUSUMESKF : public BaseEstimator {
public:
    Chi2CUSUMESKF(double drift = 0.5, double threshold_h = 25.0)
        : drift_(drift), threshold_h_(threshold_h), g_cusum_(0.0), is_quarantined_(false), gravity_(0.0, 0.0, -9.80665) {
        state_ = State15();
        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 0.1;
        P_.block<3, 3>(3, 3) *= 0.1;
        P_.block<3, 3>(6, 6) *= 0.01;
        P_.block<3, 3>(9, 9) *= 0.001;
        P_.block<3, 3>(12, 12) *= 0.0001;

        acc_noise_ = 0.08;
        gyr_noise_ = 0.004;
        acc_bias_noise_ = 0.0004;
        gyr_bias_noise_ = 0.00003;
    }

    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) override {
        state_.p = p0;
        state_.R = R0;
        state_.v.setZero();
    }

    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) override {
        Eigen::Vector3d acc = a_m - state_.ba;
        Eigen::Vector3d gyr = w_m - state_.bg;

        Eigen::Matrix3d dR = expSO3(gyr * dt);
        Eigen::Vector3d acc_world = state_.R * acc + gravity_;

        state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
        state_.v += acc_world * dt;
        state_.R = state_.R * dR;

        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * state_.R * skew(acc) * (dt * dt);
        F.block<3, 3>(0, 9) = -0.5 * state_.R * (dt * dt);
        F.block<3, 3>(3, 6) = -state_.R * skew(acc) * dt;
        F.block<3, 3>(3, 9) = -state_.R * dt;
        F.block<3, 3>(6, 6) = expSO3(-gyr * dt);
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
        Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt * dt * dt / 3.0);
        Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt);
        Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (gyr_noise_ * gyr_noise_ * dt);
        Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (acc_bias_noise_ * acc_bias_noise_ * dt);
        Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (gyr_bias_noise_ * gyr_bias_noise_ * dt);

        P_ = F * P_ * F.transpose() + Qd;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov) override {
        if (is_quarantined_) return false;

        Eigen::Vector3d nu = z_pos - state_.p;
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S = H * P_ * H.transpose() + R_cov;
        Eigen::LLT<Eigen::Matrix3d> llt(S);
        if (llt.info() != Eigen::Success) return false;

        Eigen::Vector3d w_nu = llt.matrixL().solve(nu);
        double d2 = w_nu.squaredNorm();

        // Sequential CUSUM statistic: g_k = max(0, g_{k-1} + d^2 - 3 - drift)
        g_cusum_ = std::max(0.0, g_cusum_ + d2 - 3.0 - drift_);
        if (g_cusum_ >= threshold_h_) {
            is_quarantined_ = true;
            return false;
        }

        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
        K = llt.solve(K.transpose()).transpose();

        Eigen::Matrix<double, 15, 1> delta_x = K * nu;
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_cov * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        state_.p += delta_x.segment<3>(0);
        state_.v += delta_x.segment<3>(3);
        state_.R = state_.R * expSO3(delta_x.segment<3>(6));
        state_.ba += delta_x.segment<3>(9);
        state_.bg += delta_x.segment<3>(12);

        return true;
    }

    const State15& getState() const override { return state_; }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const override { return P_; }
    std::string getName() const override { return "B4_Chi2CUSUMESKF"; }
    bool isFaultDetected() const override { return is_quarantined_; }
    double getCusumScore() const { return g_cusum_; }

private:
    double drift_;
    double threshold_h_;
    double g_cusum_;
    bool is_quarantined_;
    State15 state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Vector3d gravity_;
    double acc_noise_, gyr_noise_, acc_bias_noise_, gyr_bias_noise_;
};

// =========================================================================
// Baseline 5: Static Split-Conformal ESKF (Offline Calibration, No Online ACI)
// =========================================================================
class StaticConformalESKF : public BaseEstimator {
public:
    StaticConformalESKF(double static_quantile = 2.8)
        : q_static_(static_quantile), gravity_(0.0, 0.0, -9.80665) {
        state_ = State15();
        P_.setIdentity();
        P_.block<3, 3>(0, 0) *= 0.1;
        P_.block<3, 3>(3, 3) *= 0.1;
        P_.block<3, 3>(6, 6) *= 0.01;
        P_.block<3, 3>(9, 9) *= 0.001;
        P_.block<3, 3>(12, 12) *= 0.0001;

        acc_noise_ = 0.08;
        gyr_noise_ = 0.004;
        acc_bias_noise_ = 0.0004;
        gyr_bias_noise_ = 0.00003;
    }

    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) override {
        state_.p = p0;
        state_.R = R0;
        state_.v.setZero();
    }

    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) override {
        Eigen::Vector3d acc = a_m - state_.ba;
        Eigen::Vector3d gyr = w_m - state_.bg;

        Eigen::Matrix3d dR = expSO3(gyr * dt);
        Eigen::Vector3d acc_world = state_.R * acc + gravity_;

        state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
        state_.v += acc_world * dt;
        state_.R = state_.R * dR;

        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * state_.R * skew(acc) * (dt * dt);
        F.block<3, 3>(0, 9) = -0.5 * state_.R * (dt * dt);
        F.block<3, 3>(3, 6) = -state_.R * skew(acc) * dt;
        F.block<3, 3>(3, 9) = -state_.R * dt;
        F.block<3, 3>(6, 6) = expSO3(-gyr * dt);
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 15, 15> Qd = Eigen::Matrix<double, 15, 15>::Zero();
        Qd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt * dt * dt / 3.0);
        Qd.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (acc_noise_ * acc_noise_ * dt);
        Qd.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (gyr_noise_ * gyr_noise_ * dt);
        Qd.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (acc_bias_noise_ * acc_bias_noise_ * dt);
        Qd.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (gyr_bias_noise_ * gyr_bias_noise_ * dt);

        P_ = F * P_ * F.transpose() + Qd;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov) override {
        Eigen::Vector3d nu = z_pos - state_.p;
        Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S = H * P_ * H.transpose() + R_cov;
        Eigen::LLT<Eigen::Matrix3d> llt(S);
        if (llt.info() != Eigen::Success) return false;

        Eigen::Vector3d w_nu = llt.matrixL().solve(nu);
        double s_k = w_nu.norm();

        // Static inflation: phi = (s / q_static)^2 if s > q_static
        double phi = 1.0;
        if (s_k > q_static_) {
            phi = (s_k / q_static_) * (s_k / q_static_);
        }

        Eigen::Matrix3d R_tilde = phi * R_cov;
        Eigen::Matrix3d S_tilde = H * P_ * H.transpose() + R_tilde;
        Eigen::LLT<Eigen::Matrix3d> lltT(S_tilde);
        if (lltT.info() != Eigen::Success) return false;

        Eigen::Matrix<double, 15, 3> K = P_ * H.transpose();
        K = lltT.solve(K.transpose()).transpose();

        Eigen::Matrix<double, 15, 1> delta_x = K * nu;
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_tilde * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        state_.p += delta_x.segment<3>(0);
        state_.v += delta_x.segment<3>(3);
        state_.R = state_.R * expSO3(delta_x.segment<3>(6));
        state_.ba += delta_x.segment<3>(9);
        state_.bg += delta_x.segment<3>(12);

        return true;
    }

    const State15& getState() const override { return state_; }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const override { return P_; }
    std::string getName() const override { return "B5_StaticConformalESKF"; }

private:
    double q_static_;
    State15 state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Vector3d gravity_;
    double acc_noise_, gyr_noise_, acc_bias_noise_, gyr_bias_noise_;
};

// =========================================================================
// Proposed Method: ConESKF-FDI (Online ACI-R + E-Martingale + Pose Tubes)
// =========================================================================
class ConESKFFDIEstimator : public BaseEstimator {
public:
    ConESKFFDIEstimator() {
        filter_ = ConESKFFilter();
    }

    void initialize(const Eigen::Vector3d& p0, const Eigen::Matrix3d& R0) override {
        filter_.initialize(p0, R0);
    }

    void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt) override {
        filter_.predictIMU(a_m, w_m, dt);
    }

    bool updatePosition(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov) override {
        return filter_.updatePosition(z_pos, R_cov, "primary");
    }
    bool updatePositionChannel(const Eigen::Vector3d& z_pos, const Eigen::Matrix3d& R_cov, const std::string& modality_id) override {
        return filter_.updatePosition(z_pos, R_cov, modality_id);
    }
    bool isFaultDetected() const override {
        return const_cast<ConESKFFilter&>(filter_).isChannelQuarantined("gnss");
    }
    double getMartingaleWealth(const std::string& modality_id) const {
        return filter_.getMartingaleWealth(modality_id);
    }

    const State15& getState() const override { return filter_.getState(); }
    const Eigen::Matrix<double, 15, 15>& getCovariance() const override { return filter_.getCovariance(); }
    std::string getName() const override { return "B6_Proposed_ConESKF_FDI"; }

private:
    ConESKFFilter filter_;
};

} // namespace coneskf
