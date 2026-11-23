#!/usr/bin/env python3
"""
generate_figure1_rich.py (Refined Edition)
Generates an exhaustive, beautifully rendered, publication-grade Figure 1
with exact boundary routing, embedded visual mini-plots, and zero text/shape overlaps.
Strict IEEE Transactions two-column styling.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D

# Configuration for IEEE Transactions
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 8.0,
    'axes.labelsize': 7.5,
    'axes.titlesize': 8.0,
    'legend.fontsize': 6.2,
    'xtick.labelsize': 6.0,
    'ytick.labelsize': 6.0,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'mathtext.fontset': 'cm'
})

# Color palette
COLOR_NAVY      = "#0B3C5D"
COLOR_SLATE     = "#328CC1"
COLOR_GOLD      = "#D9B310"
COLOR_DARK_RED  = "#982649"
COLOR_GREEN     = "#008855"
COLOR_PURPLE    = "#6B4C9A"
COLOR_GREY      = "#555555"
COLOR_LIGHT_BG  = "#F7F9FA"
COLOR_ORANGE    = "#E69F00"
COLOR_BLACK     = "#000000"
COLOR_BLUE      = "#0072B2"
COLOR_VERMILION = "#D55E00"

FIG_DIR = "/home/jarvis/coneskf_fdi_ws/results/figures"
os.makedirs(FIG_DIR, exist_ok=True)

fig = plt.figure(figsize=(7.16, 4.3))
ax_main = fig.add_axes([0, 0, 1, 1])
ax_main.set_xlim(0, 100)
ax_main.set_ylim(0, 100)
ax_main.axis('off')

# Background canvas
bg_rect = patches.Rectangle((0, 0), 100, 100, facecolor='#FAFBFD', edgecolor='none', zorder=-10)
ax_main.add_patch(bg_rect)

def draw_container(ax, x, y, w, h, title, banner_color, bg_color='#FFFFFF', border_color='#CCCCCC', lw=1.2):
    box = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1,rounding_size=1.2",
                                 facecolor=bg_color, edgecolor=border_color, linewidth=lw, zorder=1)
    ax.add_patch(box)
    banner = patches.FancyBboxPatch((x, y + h - 4.2), w, 4.2, boxstyle="round,pad=0.05,rounding_size=0.8",
                                   facecolor=banner_color, edgecolor=banner_color, linewidth=0.5, zorder=2)
    ax.add_patch(banner)
    ax.text(x + w/2.0, y + h - 2.1, title, ha='center', va='center', fontsize=7.6,
            fontweight='bold', color='white', zorder=3)

def draw_arrow(ax, x1, y1, x2, y2, text='', text_pos=0.5, text_offset=(0, 2.0), color='#333333', lw=1.1, rad=0.0):
    style = "Simple,tail_width=0.7,head_width=3.6,head_length=4.2"
    kw = dict(arrowstyle=style, color=color, linewidth=lw)
    arrow = patches.FancyArrowPatch((x1, y1), (x2, y2), connectionstyle=f"arc3,rad={rad}", zorder=10, **kw)
    ax.add_patch(arrow)
    if text:
        tx = x1 + (x2 - x1) * text_pos + text_offset[0]
        ty = y1 + (y2 - y1) * text_pos + text_offset[1]
        ax.text(tx, ty, text, ha='center', va='center', fontsize=6.0, fontweight='bold',
                color=color, bbox=dict(boxstyle="round,pad=0.15", facecolor='white', edgecolor=color, linewidth=0.5, alpha=0.92), zorder=11)

# ==============================================================================
# CONTAINER 1: Sensing & Contested Environment [x: 1.5 -> 27.5, y: 52 -> 97]
# ==============================================================================
draw_container(ax_main, 1.5, 52.0, 26.0, 45.0, "1. Sensing & Contested Environment", COLOR_NAVY, '#F4F7F9', COLOR_NAVY)

# Sub-block: UAV & MEMS IMU
box_imu = patches.FancyBboxPatch((2.8, 77.0), 23.4, 14.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                facecolor='#FFFFFF', edgecolor='#328CC1', linewidth=1.0, zorder=2)
ax_main.add_patch(box_imu)
ax_main.text(14.5, 88.5, "UAV Platform & MEMS IMU", ha='center', va='center', fontsize=7.2, fontweight='bold', color='#0B3C5D', zorder=3)
ax_main.text(14.5, 84.0, "High-Rate Kinematics: 200 Hz\nAccelerometers [a_m] & Gyros [w_m]", ha='center', va='center', fontsize=6.0, color='#333333', zorder=3)
ax_main.text(14.5, 79.5, r"Body Frame $F_B \to$ Nav Frame $F_N$", ha='center', va='center', fontsize=5.8, fontstyle='italic', color='#555555', zorder=3)

# Sub-block: Contested Primary GNSS
box_gnss = patches.FancyBboxPatch((2.8, 54.0), 11.2, 21.0, boxstyle="round,pad=0.1,rounding_size=0.8",
                                 facecolor='#FFF6F6', edgecolor='#D55E00', linewidth=1.0, zorder=2)
ax_main.add_patch(box_gnss)
ax_main.text(8.4, 72.5, "Primary GNSS", ha='center', va='center', fontsize=6.8, fontweight='bold', color='#D55E00', zorder=3)
ax_main.text(8.4, 68.2, "Contested Channel", ha='center', va='center', fontsize=5.8, fontweight='semibold', color='#982649', zorder=3)
ax_main.text(8.4, 63.8, "Stealth Ramp:\n0.15 m/s Drift", ha='center', va='center', fontsize=5.6, color='#333333', zorder=3)
ax_main.text(8.4, 57.5, "Heavy-Tail NLoS:\n15% Cauchy", ha='center', va='center', fontsize=5.6, color='#555555', zorder=3)

# Sub-block: Secondary Aiding (UWB / Flow)
box_uwb = patches.FancyBboxPatch((15.0, 54.0), 11.2, 21.0, boxstyle="round,pad=0.1,rounding_size=0.8",
                                facecolor='#F4FAF6', edgecolor='#008855', linewidth=1.0, zorder=2)
ax_main.add_patch(box_uwb)
ax_main.text(20.6, 72.5, "Secondary Aiding", ha='center', va='center', fontsize=6.8, fontweight='bold', color='#008855', zorder=3)
ax_main.text(20.6, 68.2, "UWB / Optical Flow", ha='center', va='center', fontsize=5.8, fontweight='semibold', color='#0B3C5D', zorder=3)
ax_main.text(20.6, 63.8, "Local Ranging\nRelative Tracking", ha='center', va='center', fontsize=5.6, color='#333333', zorder=3)
ax_main.text(20.6, 57.5, "Safe Fallback Path\nCentimeter Mode", ha='center', va='center', fontsize=5.6, color='#008855', zorder=3)

# ==============================================================================
# CONTAINER 2: Manifold Kinematics & Innovation Whitening [x: 30.5 -> 63.5, y: 52 -> 97]
# ==============================================================================
draw_container(ax_main, 30.5, 52.0, 33.0, 45.0, "2. Manifold Kinematics & Innovation Whitening", '#328CC1', '#FFFFFF', '#328CC1')

# Sub-block: SO(3) Manifold Kinematics
box_lie = patches.FancyBboxPatch((31.8, 77.0), 30.4, 14.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                facecolor='#F9FBFC', edgecolor='#328CC1', linewidth=1.0, zorder=2)
ax_main.add_patch(box_lie)
ax_main.text(47.0, 88.5, r"$\mathrm{SO}(3)$ Manifold Kinematics & IMU Propagation", ha='center', va='center', fontsize=6.8, fontweight='bold', color='#0B3C5D', zorder=3)
ax_main.text(47.0, 84.0, r"True State on Manifold:  $\mathbf{x} = [\mathbf{p}, \mathbf{v}, \mathbf{R} \in \mathrm{SO}(3), \mathbf{b}_a, \mathbf{b}_g]^T \in \mathcal{M}$", ha='center', va='center', fontsize=5.8, color='#222222', zorder=3)
ax_main.text(47.0, 79.5, r"Discrete Propagation:  $\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_{d,k}$", ha='center', va='center', fontsize=5.8, color='#0072B2', zorder=3)

# Sub-block: Innovation Whitening & Nonconformity Scoring
box_whitening = patches.FancyBboxPatch((31.8, 54.0), 30.4, 21.0, boxstyle="round,pad=0.1,rounding_size=0.8",
                                      facecolor='#FAF8FC', edgecolor='#6B4C9A', linewidth=1.0, zorder=2)
ax_main.add_patch(box_whitening)
ax_main.text(47.0, 72.2, "Non-Parametric Innovation Whitening Engine", ha='center', va='center', fontsize=6.8, fontweight='bold', color='#6B4C9A', zorder=3)
ax_main.text(47.0, 67.8, r"Raw Measurement Innovation:  $\nu_k = \mathbf{z}_k - \hat{\mathbf{p}}_{k|k-1}$", ha='center', va='center', fontsize=6.0, color='#222222', zorder=3)
ax_main.text(47.0, 63.5, r"Cholesky Factorization:  $\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}_k = \mathbf{L}_k \mathbf{L}_k^T$", ha='center', va='center', fontsize=5.8, color='#333333', zorder=3)
ax_main.text(47.0, 59.2, r"Whitened Nonconformity:  $s_k = \|\mathbf{L}_k^{-1} \nu_k\|_2$", ha='center', va='center', fontsize=6.8, fontweight='bold', color='#982649', zorder=3)
ax_main.text(47.0, 55.6, "Distribution-Free Ranking  |  No Gaussian Assumptions", ha='center', va='center', fontsize=5.5, fontstyle='italic', color='#555555', zorder=3)

# Clear boundary connections from Container 1 to Container 2
draw_arrow(ax_main, 27.5, 84.5, 30.5, 84.5, text="[a, w]", text_pos=0.5, color='#0B3C5D')
draw_arrow(ax_main, 27.5, 64.5, 30.5, 64.5, text="z_aiding", text_pos=0.5, color='#D55E00')

# ==============================================================================
# CONTAINER 3: Anytime-Valid Betting Supermartingale (FDI Engine) [x: 66.5 -> 98.5, y: 52 -> 97]
# ==============================================================================
draw_container(ax_main, 66.5, 52.0, 32.0, 45.0, "3. Anytime-Valid Martingale Isolation (FDI)", '#D55E00', '#FFFDFB', '#D55E00')

# Inset Axes for Wealth Martingale
ax_mart = fig.add_axes([0.685, 0.535, 0.28, 0.17])
np.random.seed(42)
t_steps = np.linspace(0, 50, 100)
wealth_h0 = np.cumprod(1.0 + 0.16 * (1.0 - 2.0 * np.random.uniform(0.35, 0.65, 100)))
wealth_h1 = np.ones(100)
wealth_h1[:60] = wealth_h0[:60]
for k in range(60, 100):
    p_spoof = np.random.uniform(0.01, 0.08)
    wealth_h1[k] = wealth_h1[k-1] * (1.0 + 0.16 * (1.0 - 2.0 * p_spoof))

ax_mart.plot(t_steps, np.log10(np.maximum(0.1, wealth_h0)), color=COLOR_GREEN, linewidth=1.1, label=r'$H_0$ Safe ($< 25$)')
ax_mart.plot(t_steps, np.log10(np.maximum(0.1, wealth_h1)), color=COLOR_DARK_RED, linewidth=1.4, label=r'$H_1$ Spoofing $\log_{10}(M_k)$')
ax_mart.axhline(2.0, color='#982649', linestyle=':', linewidth=1.0, label=r'Ville: $1/\alpha_{\mathrm{FAR}}=100$')
ax_mart.axvline(30.0, color=COLOR_ORANGE, linestyle='--', linewidth=0.8)
ax_mart.text(31.0, 2.3, "Quarantine Trigger\n(TTD = 4.05s)", fontsize=5.5, color='#982649', fontweight='bold')
ax_mart.set_xlim([0, 50])
ax_mart.set_ylim([-0.5, 3.2])
ax_mart.set_xlabel("Flight Steps $k$", labelpad=1)
ax_mart.set_ylabel(r"$\log_{10}(M_k)$", labelpad=1)
ax_mart.legend(loc='upper left', framealpha=0.85, fontsize=5.2)
ax_mart.grid(True)

ax_main.text(82.5, 88.5, "Online Betting on Conformal p-values", ha='center', va='center', fontsize=6.8, fontweight='bold', color='#982649', zorder=3)
ax_main.text(82.5, 84.5, r"Conformal p-value:  $p_k = [ \sum \mathbb{I}(s_i > s_k) + U_k ] / (|\mathcal{H}_k| + 1)$", ha='center', va='center', fontsize=5.8, color='#222222', zorder=3)
ax_main.text(82.5, 80.5, r"Capital Payoff (e-value):  $e_k = 1 + \epsilon_{\mathrm{bet}} (1 - 2 p_k)$", ha='center', va='center', fontsize=5.8, color='#0B3C5D', zorder=3)
ax_main.text(82.5, 76.5, r"Supermartingale:  $M_k = \prod_{i=1}^k e_i   (\mathbb{P}(\exists k: M_k \geq \alpha^{-1}) \leq \alpha)$", ha='center', va='center', fontsize=5.8, fontweight='bold', color='#982649', zorder=3)

# Clear boundary connection from Container 2 to Container 3
draw_arrow(ax_main, 63.5, 64.5, 66.5, 64.5, text="s_k", text_pos=0.5, color='#6B4C9A')

# ==============================================================================
# CONTAINER 4: Adaptive Conformal Redescending Filter (ACI-R) [x: 1.5 -> 47.5, y: 3 -> 48]
# ==============================================================================
draw_container(ax_main, 1.5, 3.0, 46.0, 45.0, "4. Adaptive Conformal Redescending Filter (ACI-R)", COLOR_GREEN, '#FFFFFF', COLOR_GREEN)

# Sub-block: Redescending Weight Formulation
box_acir_formula = patches.FancyBboxPatch((2.8, 26.5), 43.4, 14.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                         facecolor='#F4FAF6', edgecolor=COLOR_GREEN, linewidth=1.0, zorder=2)
ax_main.add_patch(box_acir_formula)
ax_main.text(24.5, 38.0, "Closed-Form ACI-R Covariance Inflation Function", ha='center', va='center', fontsize=7.2, fontweight='bold', color='#008855', zorder=3)
ax_main.text(24.5, 33.5, r"$\phi_k = 1.0  \text{ if } s_k \leq \hat{q}_{10\%}   \mid   1.0 + \kappa \left( \frac{s_k - \hat{q}_{10}}{\hat{q}_1 - \hat{q}_{10}} \right)^2   \mid   \infty \text{ (Reject)}$", ha='center', va='center', fontsize=5.8, color='#222222', zorder=3)
ax_main.text(24.5, 29.5, r"Inflated Covariance:  $\tilde{\mathbf{R}}_k = \phi_k \mathbf{R}_k   \Rightarrow   \mathbf{K}_k(\phi_k) \to \mathbf{0}  \text{ as } \|\mathbf{v}_k\| \to \infty$", ha='center', va='center', fontsize=6.0, fontweight='bold', color='#008855', zorder=3)

# Inset Axes for ACI-R Influence Curve
ax_acir = fig.add_axes([0.05, 0.065, 0.40, 0.17])
s_grid = np.linspace(0, 6, 200)
psi_lin = s_grid
psi_hub = np.where(s_grid <= 1.345, s_grid, 1.345)
phi_grid = np.ones_like(s_grid)
mask = (s_grid > 2.0) & (s_grid <= 3.8)
phi_grid[mask] = 1.0 + 8.0 * ((s_grid[mask] - 2.0) / 1.8)**2
phi_grid[s_grid > 3.8] = 1e6
psi_acir = s_grid / phi_grid

ax_acir.plot(s_grid, psi_lin, color=COLOR_GREY, linestyle=':', linewidth=1.1, label=r'Linear $\chi^2$ (Unbounded)')
ax_acir.plot(s_grid, psi_hub, color=COLOR_ORANGE, linestyle='--', linewidth=1.2, label='Huber (Saturated)')
ax_acir.plot(s_grid, psi_acir, color=COLOR_GREEN, linewidth=1.6, label='Proposed ACI-R (Redescending)')
ax_acir.axvline(3.8, color='#982649', linestyle='-.', linewidth=0.8, label='Glitch Cutoff')
ax_acir.set_xlim([0, 6.0])
ax_acir.set_ylim([0, 3.5])
ax_acir.set_xlabel("Whitened Innovation $s_k$", labelpad=1)
ax_acir.set_ylabel(r"Influence $\psi(s)$", labelpad=1)
ax_acir.legend(loc='upper left', framealpha=0.85, fontsize=5.2)
ax_acir.grid(True)

# ==============================================================================
# CONTAINER 5: Certified Spatial Pose Tubes & Safety Reachability [x: 51.5 -> 98.5, y: 3 -> 48]
# ==============================================================================
draw_container(ax_main, 51.5, 3.0, 47.0, 45.0, "5. Certified Pose Tubes & Safety Reachability", COLOR_PURPLE, '#FAF9FC', COLOR_PURPLE)

# Sub-block: Pose Tube Ellipsoid Formula
box_tube_formula = patches.FancyBboxPatch((52.8, 26.5), 44.4, 14.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                         facecolor='#FFFFFF', edgecolor=COLOR_PURPLE, linewidth=1.0, zorder=2)
ax_main.add_patch(box_tube_formula)
ax_main.text(75.0, 38.0, "Non-Asymptotic Conformal Bounding Ellipsoids", ha='center', va='center', fontsize=7.2, fontweight='bold', color='#6B4C9A', zorder=3)
ax_main.text(75.0, 33.5, r"Certified Tube:  $\mathcal{C}_k = \{ \mathbf{p} \in \mathbb{R}^3 : (\mathbf{p} - \hat{\mathbf{p}}_k)^T \mathbf{P}_{p,k}^{-1} (\mathbf{p} - \hat{\mathbf{p}}_k) \leq \rho_k^2 \}$", ha='center', va='center', fontsize=6.0, color='#222222', zorder=3)
ax_main.text(75.0, 29.5, r"Guaranteed Coverage:  $\mathbb{P}(\mathbf{p}_k^{\mathrm{true}} \in \mathcal{C}_k) \geq 1 - \alpha_{\mathrm{pose}}  \quad (\text{Real Flight: } 90.6\% \text{ at } 95\%)$", ha='center', va='center', fontsize=6.0, fontweight='bold', color='#008855', zorder=3)

# Inset Axes for 3D Ellipsoid Wireframe
ax_tube = fig.add_axes([0.55, 0.05, 0.40, 0.20], projection='3d')
u = np.linspace(0, 2 * np.pi, 20)
v = np.linspace(0, np.pi, 10)
rx, ry, rz = 1.2, 0.8, 0.5
x_ell = rx * np.outer(np.cos(u), np.sin(v))
y_ell = ry * np.outer(np.sin(u), np.sin(v))
z_ell = rz * np.outer(np.ones_like(u), np.cos(v))

ax_tube.plot_wireframe(x_ell, y_ell, z_ell, color=COLOR_PURPLE, alpha=0.35, linewidth=0.6)
ax_tube.scatter([0], [0], [0], color=COLOR_GREEN, s=20, label=r'Estimated Pose $\hat{p}_k$')
ax_tube.scatter([0.2], [-0.15], [0.1], color=COLOR_BLACK, marker='x', s=30, label=r'Ground Truth $p_k^{\mathrm{true}}$')
ax_tube.set_xlim([-1.5, 1.5])
ax_tube.set_ylim([-1.5, 1.5])
ax_tube.set_zlim([-1.0, 1.0])
ax_tube.set_xlabel('X [m]', labelpad=-11, fontsize=5.2)
ax_tube.set_ylabel('Y [m]', labelpad=-11, fontsize=5.2)
ax_tube.set_zlabel('Z [m]', labelpad=-11, fontsize=5.2)
ax_tube.tick_params(labelsize=4.8, pad=-4)
ax_tube.view_init(elev=20, azim=45)
ax_tube.legend(loc='upper right', framealpha=0.85, fontsize=5.0)

# ==============================================================================
# Inter-Container Control & Data Flow Connections (Strictly in Empty Margins)
# ==============================================================================
# 1. Innovation Whitening s_k -> ACI-R Covariance Inflation (vertical margin between top & bottom rows)
draw_arrow(ax_main, 47.0, 52.0, 30.0, 48.0, text="s_k Score", text_pos=0.5, color='#6B4C9A', rad=0.1)

# 2. Quarantine Decision from Martingale Engine -> FDI Isolation Action
draw_arrow(ax_main, 82.5, 52.0, 75.0, 48.0, text="Quarantine Trigger (M_k >= 100)", text_pos=0.45, color='#982649', rad=-0.1)

# 3. ACI-R State Correction -> Pose Tube (horizontal margin between Container 4 & 5)
draw_arrow(ax_main, 47.5, 25.0, 51.5, 25.0, text="p_hat, P_pos", text_pos=0.5, color='#008855')

# Save Refined Figure 1
pdf_path = os.path.join(FIG_DIR, "fig1_coneskf_architecture.pdf")
png_path = os.path.join(FIG_DIR, "fig1_coneskf_architecture.png")
plt.savefig(pdf_path, bbox_inches='tight')
plt.savefig(png_path, bbox_inches='tight')
plt.close()
print(f"Refined Figure 1 successfully generated:\n  {pdf_path}\n  {png_path}")
