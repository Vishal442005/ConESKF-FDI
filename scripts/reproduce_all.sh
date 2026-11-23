#!/usr/bin/env bash
# ==============================================================================
# reproduce_all.sh
# End-to-End One-Click Reproducibility Pipeline for ConESKF-FDI
# ==============================================================================
set -e

echo "=== [1/4] Sourcing ROS 2 and Building Workspace Packages ==="
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_DIR}"

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

echo "=== [2/4] Executing EuRoC MAV & ALFA Benchmark Pipeline ==="
python3 "${REPO_DIR}/src/coneskf_benchmark/run_alfa_benchmark.py"

echo "=== [3/4] Generating All Publication Figures (Fig 1 - Fig 6) ==="
python3 "${REPO_DIR}/src/coneskf_benchmark/generate_all_figures_large_text.py"

echo "=== [4/4] Compiling 10-Page IEEE Transactions Manuscript ==="
cd "${REPO_DIR}/paper"
pdflatex -interaction=nonstopmode coneskf_fdi_manuscript.tex
pdflatex -interaction=nonstopmode coneskf_fdi_manuscript.tex

echo "=== Reproducibility Pipeline Completed Successfully! ==="
echo "Manuscript compiled: ${REPO_DIR}/paper/coneskf_fdi_manuscript.pdf"
