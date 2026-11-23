#pragma once

#include <vector>
#include <deque>
#include <random>
#include <algorithm>
#include <cmath>
#include <string>

namespace coneskf {

class ConformalBettingMartingale {
public:
    ConformalBettingMartingale(
        size_t window_size = 200,
        size_t min_calibration_samples = 100,
        double epsilon_bet = 0.16,
        double lambda_decay = 0.0,
        double alpha_far = 0.01,
        uint64_t seed = 42
    ) : window_size_(window_size),
        min_calibration_samples_(min_calibration_samples),
        epsilon_bet_(epsilon_bet),
        lambda_decay_(lambda_decay),
        alpha_far_(alpha_far),
        threshold_(1.0 / alpha_far),
        wealth_(1.0),
        is_quarantined_(false),
        rng_(seed),
        unif_dist_(0.0, 1.0) {}

    void reset() {
        history_.clear();
        wealth_ = 1.0;
        is_quarantined_ = false;
    }

    double update(double s_k) {
        if (is_quarantined_) {
            return wealth_;
        }

        if (history_.size() < min_calibration_samples_) {
            history_.push_back(s_k);
            return wealth_;
        }

        size_t count_greater = 0;
        size_t count_equal = 0;
        for (double s_past : history_) {
            if (s_past > s_k) {
                count_greater++;
            } else if (std::abs(s_past - s_k) < 1e-9) {
                count_equal++;
            }
        }

        double u_k = unif_dist_(rng_);
        double p_k = (static_cast<double>(count_greater) + u_k * (static_cast<double>(count_equal) + 1.0)) 
                     / (static_cast<double>(history_.size()) + 1.0);

        p_k = std::clamp(p_k, 1e-6, 1.0 - 1e-6);

        // Capital allocation betting payoff (e-value): E_{H0}[e_k | F_{k-1}] = 1
        double e_k = 1.0 + epsilon_bet_ * (1.0 - 2.0 * p_k);

        // Ville's anytime-valid test supermartingale update
        wealth_ *= e_k;

        // Ville's Stopping Time Rule: quarantine if M_k >= 1 / alpha_FAR
        if (wealth_ >= threshold_) {
            is_quarantined_ = true;
        } else {
            // Only add unquarantined healthy nonconformity scores to calibration history
            history_.push_back(s_k);
            if (history_.size() > window_size_) {
                history_.pop_front();
            }
        }

        return wealth_;
    }

    bool isQuarantined() const { return is_quarantined_; }
    double getWealth() const { return wealth_; }
    double getThreshold() const { return threshold_; }

private:
    size_t window_size_;
    size_t min_calibration_samples_;
    double epsilon_bet_;
    double lambda_decay_;
    double alpha_far_;
    double threshold_;
    double wealth_;
    bool is_quarantined_;
    std::deque<double> history_;
    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> unif_dist_;
};

} // namespace coneskf
