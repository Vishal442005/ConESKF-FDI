#pragma once

#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>

namespace coneskf {

class CertifiedPoseTube {
public:
    CertifiedPoseTube(
        double alpha_pose = 0.05,
        size_t calibration_window = 120
    ) : alpha_pose_(alpha_pose),
        target_coverage_(1.0 - alpha_pose),
        window_size_(calibration_window),
        rho_k_(3.0),
        total_ticks_(0),
        covered_ticks_(0) {}

    // Evaluates ground truth against predicted position and covariance sub-block
    // Returns true if p_gt is inside the certified ellipsoid E_pos,k
    bool update(
        const Eigen::Vector3d& p_est,
        const Eigen::Matrix3d& P_pos,
        const Eigen::Vector3d& p_gt,
        double* out_radius = nullptr,
        double* out_volume = nullptr
    ) {
        total_ticks_++;

        // 1. Compute normalized Mahalanobis error: epsilon = ||P_pos^{-1/2} (p_gt - p_est)||_2
        Eigen::Vector3d err = p_gt - p_est;
        Eigen::LLT<Eigen::Matrix3d> llt(P_pos);
        double epsilon = 0.0;
        if (llt.info() == Eigen::Success) {
            Eigen::Vector3d whitened_err = llt.matrixL().solve(err);
            epsilon = whitened_err.norm();
        } else {
            epsilon = err.norm();
        }

        // 2. Check empirical coverage with current radius rho_k
        bool is_covered = (epsilon <= rho_k_);
        if (is_covered) {
            covered_ticks_++;
        }

        // 3. Update rolling calibration window
        err_history_.push_back(epsilon);
        if (err_history_.size() > window_size_) {
            err_history_.pop_front();
        }

        // 4. Update online calibrated radius rho_k: (1 - alpha_pose) quantile
        std::vector<double> sorted_err(err_history_.begin(), err_history_.end());
        std::sort(sorted_err.begin(), sorted_err.end());
        size_t q_idx = static_cast<size_t>(std::ceil(target_coverage_ * sorted_err.size()));
        q_idx = std::clamp(q_idx, static_cast<size_t>(0), sorted_err.size() - 1);
        rho_k_ = sorted_err[q_idx];

        // 5. Certified Ellipsoid Volume: V = (4/3) * pi * rho_k^3 * sqrt(det(P_pos))
        double detP = P_pos.determinant();
        double volume = (4.0 / 3.0) * M_PI * std::pow(rho_k_, 3.0) * std::sqrt(std::max(1e-12, detP));

        if (out_radius) *out_radius = rho_k_;
        if (out_volume) *out_volume = volume;

        return is_covered;
    }

    double getRadius() const { return rho_k_; }
    double getEmpiricalCoverage() const {
        return (total_ticks_ > 0) ? (static_cast<double>(covered_ticks_) / total_ticks_) : 1.0;
    }
    double getTargetCoverage() const { return target_coverage_; }
    size_t getTotalTicks() const { return total_ticks_; }

private:
    double alpha_pose_;
    double target_coverage_;
    size_t window_size_;
    double rho_k_;
    size_t total_ticks_;
    size_t covered_ticks_;
    std::deque<double> err_history_;
};

} // namespace coneskf
