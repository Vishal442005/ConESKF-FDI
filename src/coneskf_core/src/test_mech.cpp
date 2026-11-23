
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <random>

void testMechanisms() {
    // Let's generate:
    // Phase 1: 600 steps of H0 (p_k ~ Uniform(0, 1))
    // Phase 2: 200 steps of H1 (ramp spoofing where p_k shifts towards 0: p_k ~ Beta(0.5, 2.0))
    // Phase 3: 600 steps of H0
    
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    auto sim_pk = [&](size_t step) {
        if (step >= 600 && step < 800) {
            // Under attack, ramp slowly pushes p_k lower
            double ramp = (step - 600) / 200.0;
            double p = unif(rng);
            return p * (1.0 - 0.8 * ramp); // p_k gradually shifts towards 0
        } else {
            return unif(rng);
        }
    };

    double eps = 0.20;

    // Mech 1: Classic Martingale with Floor at 0.1 and negative drift
    // Mech 2: Windowed Martingale (product over last W_test steps: M_k = prod_{j=k-L+1}^k e_j)
    // Mech 3: CUSUM Log-Martingale: g_k = max(0, g_{k-1} + ln(e_k) - drift)
    // Mech 4: Reset Martingale: if M_k < 0.05, reset M_k = 1.0; if M_k >= 100, quarantine!

    size_t L_win = 40; // 40 steps at 20 Hz = 2.0 seconds
    std::deque<double> e_win;

    double m_floor = 1.0;
    double g_cusum = 0.0;
    double m_reset = 1.0;

    double max_h0_win = 0, max_h0_floor = 0, max_h0_cusum = 0, max_h0_reset = 0;
    double h1_detect_win = -1, h1_detect_cusum = -1, h1_detect_reset = -1;

    for (size_t k = 0; k < 1400; ++k) {
        double p_k = sim_pk(k);
        double e_k = 1.0 + eps * (1.0 - 2.0 * p_k);

        // Mech 2: Windowed Martingale
        e_win.push_back(e_k);
        if (e_win.size() > L_win) e_win.pop_front();
        double m_w = 1.0;
        for (double e : e_win) m_w *= e;

        // Mech 3: Log-martingale CUSUM with small safety drift (e.g. 0.01)
        g_cusum = std::max(0.0, g_cusum + std::log(e_k) - 0.015);

        // Mech 4: Reset Martingale
        m_reset *= e_k;
        if (m_reset < 0.1) m_reset = 1.0;

        bool in_h0 = (k < 600 || k >= 800);
        if (in_h0) {
            if (m_w > max_h0_win) max_h0_win = m_w;
            if (g_cusum > max_h0_cusum) max_h0_cusum = g_cusum;
            if (m_reset > max_h0_reset) max_h0_reset = m_reset;
        } else {
            // in H1
            if (m_w >= 100.0 && h1_detect_win < 0) h1_detect_win = (k - 600);
            if (g_cusum >= std::log(100.0) && h1_detect_cusum < 0) h1_detect_cusum = (k - 600);
            if (m_reset >= 100.0 && h1_detect_reset < 0) h1_detect_reset = (k - 600);
        }
    }

    std::cout << "Results on 1,400-step test:" << std::endl;
    std::cout << "Windowed Martingale (L=" << L_win << "): Max H0 Wealth = " << max_h0_win 
              << ", H1 Detection Delay = " << h1_detect_win << " steps" << std::endl;
    std::cout << "Log-Martingale CUSUM: Max H0 g = " << max_h0_cusum << " (thresh=" << std::log(100.0) << ")"
              << ", H1 Detection Delay = " << h1_detect_cusum << " steps" << std::endl;
    std::cout << "Reset Martingale (floor reset to 1.0): Max H0 Wealth = " << max_h0_reset 
              << ", H1 Detection Delay = " << h1_detect_reset << " steps" << std::endl;
}

int main() {
    testMechanisms();
    return 0;
}
