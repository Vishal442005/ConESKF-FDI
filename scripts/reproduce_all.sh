#!/usr/bin/env bash
# ==============================================================================
# reproduce_all.sh
# End-to-End One-Click Reproducibility Pipeline for ConESKF-FDI
# ==============================================================================
set -e

echo "=== [1/3] Sourcing ROS 2 and Building Workspace Packages ==="
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_DIR}"

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

echo "=== [2/3] Executing EuRoC MAV & ALFA Benchmark Pipeline ==="
python3 "${REPO_DIR}/src/coneskf_benchmark/run_alfa_benchmark.py"

echo "=== [3/3] Generating All Publication Figures (Fig 1 - Fig 6) ==="
python3 "${REPO_DIR}/src/coneskf_benchmark/generate_all_figures_large_text.py"

echo "=== Reproducibility Pipeline Completed Successfully! ==="
echo "All publication figures generated in ${REPO_DIR}/results/figures/"
