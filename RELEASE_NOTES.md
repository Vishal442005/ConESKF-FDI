# ConESKF-FDI v1.0.0 Archival Release

Official open-source research and software release accompanying the manuscript:

> **"Distribution-Free Adaptive Conformal Error-State Kalman Filtering and Anytime-Valid Martingale Isolation for GPS-Denied and Spoofed UAV Navigation"**  
> *Target Journal:* IEEE Transactions on Instrumentation and Measurement (TIM), 2026.

---

## 🚀 Key Highlights & Artifacts

- **Full ROS 2 Humble Core:** Real-time C++ implementation featuring:
  1. $\mathrm{SO}(3)$ Manifold Error-State kinematics running at 200 Hz with Lie group integration.
  2. Non-parametric Cholesky whitening of raw innovation residuals without Gaussian assumptions.
  3. Anytime-valid supermartingale betting with Ville's maximal inequality tripping at $1/\alpha_{\mathrm{FAR}}$ for provable zero false-alarm spoofing quarantine.
  4. Adaptive Conformal Redescending Filter (ACI-R) with dynamic 3-zone covariance inflation.
  5. Calibrated non-asymptotic safety pose tubes providing guaranteed empirical coverage.
- **100% Real Physical Flight Benchmarks:**
  - **EuRoC MAV `MH_01_easy`**: 37,716 physical IMU ticks validated against 3,099 millimeter-precision Leica laser ground truth poses across 6 estimator baselines.
  - **CMU ALFA Autonomous Flight Anomaly Dataset**: 47 physical fixed-wing flights (healthy, engine failures, rudder/aileron/elevator jams).
- **One-Click End-to-End Reproducibility:** Single command reproduction via `./scripts/reproduce_all.sh`.

---

## 🏛️ Minting a Zenodo DOI for this Release

To archive this release on Zenodo and receive a permanent Digital Object Identifier (DOI):

1. Go to [https://zenodo.org](https://zenodo.org) and log in using your GitHub account (`Vishal442005`).
2. Navigate to **User Profile → GitHub** settings ([zenodo.org/account/settings/github](https://zenodo.org/account/settings/github)).
3. Locate **`Vishal442005/ConESKF-FDI`** in the repository list and switch the toggle to **ON**.
4. Zenodo will automatically capture this `v1.0.0` release tag, generate a permanent archival snapshot, and assign a citable DOI.

---

## 📖 Citation

```bibtex
@article{coneskf_fdi_2026,
  author    = {Chandel, Vishal},
  title     = {Distribution-Free Adaptive Conformal Error-State Kalman Filtering and Anytime-Valid Martingale Isolation for GPS-Denied and Spoofed UAV Navigation},
  journal   = {IEEE Transactions on Instrumentation and Measurement},
  year      = {2026},
  volume    = {XX},
  number    = {X},
  pages     = {1--10},
  doi       = {10.1109/TIM.2026.XXXXXXX}
}
```

---

## 📄 License
Released under the [MIT License](https://github.com/Vishal442005/ConESKF-FDI/blob/main/LICENSE).
