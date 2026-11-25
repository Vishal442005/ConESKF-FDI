#!/usr/bin/env python3
"""
generate_figure1_rich_visual.py
Publication-Grade Architecture Diagram for ConESKF-FDI:
- Strictly BLACK & WHITE text everywhere (zero colored text).
- Neutral, elegant white boxes with thin charcoal borders (zero colored box fills or banners).
- Colors reserved ONLY inside graphical figures and data plots.
- ZERO text overflow, zero clipping, generous margins everywhere.
- High visual informativeness: visual schematics for sensors, Lie manifold,
  whitening transform, martingale betting, 3-zone inflation, and certified reachability tube.
"""

import os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
from mpl_toolkits.mplot3d import Axes3D

RESULTS_DIR = "/home/jarvis/coneskf_fdi_ws/results"
PLOTS_DIR = os.path.join(RESULTS_DIR, "figures")
os.makedirs(PLOTS_DIR, exist_ok=True)
ARTIFACT_DIR = "/home/jarvis/.gemini/antigravity/brain/eab93cba-b1b0-41f1-870b-d750b1d3b20d"

# Graphical Figure Colors (Reserved strictly for visual plots and diagrams)
FIG_COLOR_NAVY      = "#0B3C5D"
FIG_COLOR_SLATE     = "#328CC1"
FIG_COLOR_DARK_RED  = "#982649"
FIG_COLOR_GREEN     = "#008855"
FIG_COLOR_PURPLE    = "#6B4C9A"
FIG_COLOR_GREY      = "#666666"
FIG_COLOR_ORANGE    = "#E69F00"
FIG_COLOR_BLUE      = "#0072B2"
FIG_COLOR_VERMILION = "#D55E00"

# Neutral Box & Text Palette
TEXT_BLACK       = "#111111"
BOX_BORDER       = "#333333"
BOX_BORDER_LIGHT = "#777777"
BOX_BG           = "#FFFFFF"
HEADER_BG        = "#F0F2F5"

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 8.5,
    'axes.labelsize': 8.5,
    'axes.titlesize': 8.5,
    'legend.fontsize': 7.0,
    'xtick.labelsize': 7.0,
    'ytick.labelsize': 7.0,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'lines.linewidth': 1.6,
    'lines.markersize': 4,
    'grid.alpha': 0.35,
    'grid.linestyle': '--',
    'mathtext.fontset': 'cm'
})

def create_rich_figure1():
    print("Generating High-Clarity, Zero-Overflow Figure 1...")
    fig = plt.figure(figsize=(7.16, 5.95))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')

    # Pure White Canvas
    ax.add_patch(patches.Rectangle((0, 0), 100, 100, facecolor='#FFFFFF', edgecolor='none', zorder=-10))

    # Helper: Clean Container with elegant neutral header banner
    def draw_container(ax, x, y, w, h, title):
        # Outer box with crisp neutral border
        box = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.12,rounding_size=1.0",
                                     facecolor=BOX_BG, edgecolor=BOX_BORDER, linewidth=1.1, zorder=1)
        ax.add_patch(box)
        # Header banner in neutral light grey with thin dividing rule
        banner = patches.FancyBboxPatch((x, y + h - 4.2), w, 4.2, boxstyle="round,pad=0.08,rounding_size=0.8",
                                       facecolor=HEADER_BG, edgecolor=BOX_BORDER, linewidth=0.8, zorder=2)
        ax.add_patch(banner)
        # Title text: STRICTLY BOLD BLACK, comfortably padded
        ax.text(x + w/2.0, y + h - 2.1, title, ha='center', va='center', fontsize=7.2,
                fontweight='bold', color=TEXT_BLACK, zorder=3)

    # Helper: Styled neutral arrow with clean white pill
    def draw_arrow(ax, x1, y1, x2, y2, text='', text_pos=0.5, text_offset=(0, 0), rad=0.0, fs=5.6):
        style = "Simple,tail_width=0.7,head_width=3.6,head_length=4.2"
        kw = dict(arrowstyle=style, color='#222222', linewidth=1.0)
        arrow = patches.FancyArrowPatch((x1, y1), (x2, y2), connectionstyle=f"arc3,rad={rad}", zorder=10, **kw)
        ax.add_patch(arrow)
        if text:
            tx = x1 + (x2 - x1) * text_pos + text_offset[0]
            ty = y1 + (y2 - y1) * text_pos + text_offset[1]
            ax.text(tx, ty, text, ha='center', va='center', fontsize=fs, fontweight='bold',
                    color=TEXT_BLACK, bbox=dict(boxstyle="round,pad=0.14", facecolor='#FFFFFF', edgecolor='#444444', linewidth=0.6, alpha=0.98), zorder=11)

    # Helper: Horizontal inter-container arrow with label positioned cleanly ABOVE the line
    def draw_h_arrow(ax, x1, x2, y, text='', text_offset_y=1.4, fs=6.0):
        style = "Simple,tail_width=0.7,head_width=3.6,head_length=4.2"
        arrow = patches.FancyArrowPatch((x1, y), (x2, y), zorder=10,
                                        arrowstyle=style, color='#222222', linewidth=1.0)
        ax.add_patch(arrow)
        if text:
            ax.text((x1 + x2)/2.0, y + text_offset_y, text, ha='center', va='bottom',
                    fontsize=fs, fontweight='bold', color=TEXT_BLACK, zorder=11)

    # ==========================================================================
    # CONTAINER 1: Sensors & Flight Dynamics [x: 1.5 -> 26.5, y: 51.5 -> 98.5]
    # ==========================================================================
    draw_container(ax, 1.5, 51.5, 25.0, 47.0, "1. Sensors & Flight Dynamics")

    # UAV Drone Schematic Box (White bg, neutral border)
    box_uav = patches.FancyBboxPatch((2.5, 75.8), 23.0, 16.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                     facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.9, zorder=2)
    ax.add_patch(box_uav)
    ax.text(14.0, 89.8, "UAV Platform & MEMS IMU", ha='center', va='center', fontsize=6.6, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # FIGURE: Quadrotor Drone Visual Icon (Colors inside figure)
    drone_cx, drone_cy = 7.0, 82.8
    ax.add_patch(patches.Circle((drone_cx, drone_cy), 1.05, facecolor=FIG_COLOR_SLATE, edgecolor=FIG_COLOR_NAVY, linewidth=1.1, zorder=4))
    arm_len = 1.7
    ax.plot([drone_cx - arm_len, drone_cx + arm_len], [drone_cy - arm_len, drone_cy + arm_len], color=FIG_COLOR_NAVY, linewidth=1.4, zorder=3)
    ax.plot([drone_cx - arm_len, drone_cx + arm_len], [drone_cy + arm_len, drone_cy - arm_len], color=FIG_COLOR_NAVY, linewidth=1.4, zorder=3)
    for dx, dy in [(-arm_len, -arm_len), (arm_len, arm_len), (-arm_len, arm_len), (arm_len, -arm_len)]:
        ax.add_patch(patches.Ellipse((drone_cx + dx, drone_cy + dy), 1.5, 0.6, angle=25, facecolor=FIG_COLOR_ORANGE, edgecolor=FIG_COLOR_NAVY, linewidth=0.8, alpha=0.9, zorder=4))
    # Body axes
    ax.annotate('', xy=(drone_cx + 2.4, drone_cy), xytext=(drone_cx, drone_cy),
                arrowprops=dict(arrowstyle="->", color=FIG_COLOR_VERMILION, lw=1.2), zorder=5)
    ax.text(drone_cx + 2.7, drone_cy, r"$X_B$", fontsize=5.8, fontweight='bold', color=FIG_COLOR_VERMILION, va='center')
    ax.annotate('', xy=(drone_cx, drone_cy + 2.3), xytext=(drone_cx, drone_cy),
                arrowprops=dict(arrowstyle="->", color=FIG_COLOR_GREEN, lw=1.2), zorder=5)
    ax.text(drone_cx, drone_cy + 2.6, r"$Y_B$", fontsize=5.8, fontweight='bold', color=FIG_COLOR_GREEN, ha='center')

    # Drone IMU text info: ALL BLACK TEXT
    ax.text(17.8, 85.0, "200 Hz Tri-axial IMU", ha='center', va='center', fontsize=6.0, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(17.8, 81.8, r"Spec. Force $\mathbf{a}_m \in \mathbb{R}^3$", ha='center', va='center', fontsize=5.6, color=TEXT_BLACK, zorder=3)
    ax.text(17.8, 78.8, r"Ang. Rate $\boldsymbol{\omega}_m \in \mathbb{R}^3$", ha='center', va='center', fontsize=5.6, color=TEXT_BLACK, zorder=3)

    # UNIFIED Measurement Aiding Box (generous padding, zero overflow)
    box_meas = patches.FancyBboxPatch((2.5, 52.8), 23.0, 21.0, boxstyle="round,pad=0.1,rounding_size=0.8",
                                      facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.9, zorder=2)
    ax.add_patch(box_meas)
    # Subtle vertical divider line at x=14.0
    ax.plot([14.0, 14.0], [53.5, 73.0], color='#CCCCCC', linestyle=':', linewidth=0.8, zorder=3)

    # Left: Primary GNSS (Contested) - center at 8.25
    ax.text(8.25, 71.8, "Primary GNSS", ha='center', va='center', fontsize=5.8, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(8.25, 69.2, "(Contested)", ha='center', va='center', fontsize=5.2, fontstyle='italic', color=TEXT_BLACK, zorder=3)

    # FIGURE: Visual attack waveform (Colors inside figure)
    t_wave = np.linspace(3.5, 13.0, 40)
    y_wave = 63.8 + 0.35 * (t_wave - 3.5) / 9.5 * 5.0 + 0.3 * np.sin(4.0 * t_wave)
    y_wave[22] += 2.2
    ax.plot(t_wave, y_wave, color=FIG_COLOR_VERMILION, linewidth=1.2, zorder=4)
    ax.scatter([t_wave[22]], [y_wave[22]], color=FIG_COLOR_DARK_RED, s=16, marker='^', zorder=5)
    # Text: ALL BLACK, concise
    ax.text(8.25, 58.2, "Stealth Ramp Drift\n(0.15 m/s) + Cauchy", ha='center', va='center', fontsize=5.0, color=TEXT_BLACK, zorder=3)
    ax.text(8.25, 54.6, r"Evades $\chi^2$ Gate", ha='center', va='center', fontsize=5.0, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # Right: Secondary Aiding (UWB / Flow) - center at 19.75
    ax.text(19.75, 71.8, "Secondary Aid", ha='center', va='center', fontsize=5.8, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(19.75, 69.2, "(UWB / Flow)", ha='center', va='center', fontsize=5.2, fontstyle='italic', color=TEXT_BLACK, zorder=3)

    # FIGURE: Visual UWB Antenna & concentric arcs (Colors inside figure)
    uwb_cx, uwb_cy = 19.75, 63.8
    ax.plot([uwb_cx - 1.2, uwb_cx, uwb_cx + 1.2], [uwb_cy - 2.0, uwb_cy + 1.2, uwb_cy - 2.0], color=FIG_COLOR_GREEN, linewidth=1.2, zorder=4)
    for r_arc in [1.2, 2.0, 2.8]:
        arc = patches.Arc((uwb_cx, uwb_cy + 1.2), r_arc, r_arc, angle=0, theta1=30, theta2=150, color=FIG_COLOR_GREEN, linewidth=0.9, linestyle='--', zorder=4)
        ax.add_patch(arc)
    # Text: ALL BLACK, concise
    ax.text(19.75, 58.2, "Local Ranging\nRelative Tracking", ha='center', va='center', fontsize=5.0, color=TEXT_BLACK, zorder=3)
    ax.text(19.75, 54.6, "Centimeter Mode", ha='center', va='center', fontsize=5.0, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # ==========================================================================
    # CONTAINER 2: Kinematics & Whitening [x: 31.0 -> 64.0, y: 51.5 -> 98.5]
    # ==========================================================================
    draw_container(ax, 31.0, 51.5, 33.0, 47.0, "2. SO(3) Kinematics & Whitening")

    # SO(3) Manifold Box (White bg, neutral border)
    box_lie = patches.FancyBboxPatch((32.0, 75.8), 31.0, 16.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                     facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.9, zorder=2)
    ax.add_patch(box_lie)
    ax.text(47.5, 89.8, r"$\mathrm{SO}(3)$ Manifold Error-State Propagation", ha='center', va='center', fontsize=6.6, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # FIGURE: Curved manifold surface and tangent plane (Colors inside figure)
    u_curve = np.linspace(33.0, 41.0, 30)
    v_curve = 79.5 + 1.1 * np.sin((u_curve - 33.0) / 8.0 * np.pi)
    ax.plot(u_curve, v_curve, color=FIG_COLOR_SLATE, linewidth=1.4, zorder=4)
    ax.text(33.3, 78.0, r"$\mathcal{M}$", fontsize=6.5, fontweight='bold', color=FIG_COLOR_SLATE)
    ax.plot([35.0, 40.7], [80.5, 83.1], color=FIG_COLOR_NAVY, linewidth=1.5, zorder=5)
    ax.text(40.1, 83.7, r"$T_{\mathbf{x}}\mathcal{M}$", fontsize=6.0, fontweight='bold', color=FIG_COLOR_NAVY)
    ax.annotate('', xy=(38.7, 82.1), xytext=(37.3, 80.5),
                arrowprops=dict(arrowstyle="->", color=FIG_COLOR_VERMILION, lw=1.2), zorder=6)
    ax.text(36.5, 82.1, r"$\mathrm{Exp}$", fontsize=5.6, color=FIG_COLOR_VERMILION)

    # Equations in Manifold box: ALL BLACK TEXT
    ax.text(53.0, 85.0, r"$\mathbf{x} = [\mathbf{p}, \mathbf{v}, \mathbf{R} \in \mathrm{SO}(3), \mathbf{b}_a, \mathbf{b}_g]^T$", ha='center', va='center', fontsize=5.6, color=TEXT_BLACK, zorder=3)
    ax.text(53.0, 81.8, r"True Error State:  $\delta \mathbf{x} \in \mathbb{R}^{15}$", ha='center', va='center', fontsize=5.6, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(53.0, 78.8, r"$\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_{d,k}$", ha='center', va='center', fontsize=5.6, color=TEXT_BLACK, zorder=3)

    # Innovation Whitening Box (White bg, neutral border)
    box_whitening = patches.FancyBboxPatch((32.0, 52.8), 31.0, 21.0, boxstyle="round,pad=0.1,rounding_size=0.8",
                                          facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.9, zorder=2)
    ax.add_patch(box_whitening)
    ax.text(47.5, 71.8, "Non-Parametric Innovation Whitening", ha='center', va='center', fontsize=6.6, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # FIGURE: Correlated Innovation Ellipse -> Cholesky Transform -> Isotropic Whitened Sphere
    # 1. Correlated Ellipse (Colors inside figure)
    ell_cx, ell_cy = 34.6, 62.6
    ell_raw = patches.Ellipse((ell_cx, ell_cy), 4.4, 2.1, angle=35, facecolor='#FBE8EB', edgecolor=FIG_COLOR_VERMILION, linewidth=1.2, zorder=3)
    ax.add_patch(ell_raw)
    ax.annotate('', xy=(ell_cx + 1.3, ell_cy + 0.8), xytext=(ell_cx, ell_cy),
                arrowprops=dict(arrowstyle="->", color=FIG_COLOR_DARK_RED, lw=1.2), zorder=4)
    # Label cleanly ABOVE ellipse (never overlapping)
    ax.text(ell_cx, ell_cy + 2.2, r"Raw $\boldsymbol{\nu}_k \sim \mathbf{S}_k$", ha='center', va='bottom', fontsize=5.2, fontweight='bold', color=TEXT_BLACK, zorder=4)

    # 2. Cholesky Factor Arrow (Colors inside figure)
    ax.annotate('', xy=(41.4, ell_cy), xytext=(37.8, ell_cy),
                arrowprops=dict(arrowstyle="->", color=FIG_COLOR_PURPLE, lw=1.4), zorder=5)
    ax.text(39.6, ell_cy + 0.9, r"$\mathbf{L}_k^{-1}$", ha='center', va='bottom', fontsize=6.0, fontweight='bold', color=FIG_COLOR_PURPLE, zorder=5)

    # 3. Whitened Sphere (Colors inside figure)
    sph_cx, sph_cy = 44.8, ell_cy
    sph_white = patches.Circle((sph_cx, sph_cy), 1.75, facecolor='#EBF5FB', edgecolor=FIG_COLOR_BLUE, linewidth=1.2, zorder=3)
    ax.add_patch(sph_white)
    ax.annotate('', xy=(sph_cx + 1.1, sph_cy + 0.75), xytext=(sph_cx, sph_cy),
                arrowprops=dict(arrowstyle="->", color=FIG_COLOR_BLUE, lw=1.2), zorder=4)
    # Label cleanly ABOVE circle (never overlapping)
    ax.text(sph_cx, ell_cy + 2.2, r"Whitened $\mathbf{L}_k^{-1} \boldsymbol{\nu}_k$", ha='center', va='bottom', fontsize=5.2, fontweight='bold', color=TEXT_BLACK, zorder=4)

    # Nonconformity score definition: ALL BLACK TEXT
    ax.text(56.2, 67.5, r"$\mathbf{S}_k = \mathbf{L}_k \mathbf{L}_k^T$", ha='center', va='center', fontsize=5.6, color=TEXT_BLACK, zorder=3)
    ax.text(56.2, 64.2, r"$s_k = \|\mathbf{L}_k^{-1} \boldsymbol{\nu}_k\|_2$", ha='center', va='center', fontsize=6.4, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(56.2, 60.8, r"Distribution-Free", ha='center', va='center', fontsize=5.3, fontstyle='italic', color=TEXT_BLACK, zorder=3)
    ax.text(56.2, 57.5, r"Window $\mathcal{H}_k$ ($N=50$)", ha='center', va='center', fontsize=5.3, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(56.2, 54.6, r"Conformal Calibration", ha='center', va='center', fontsize=5.1, color=TEXT_BLACK, zorder=3)

    # Connections from Container 1 to Container 2 (Generous 4.5 gap: x in [26.5, 31.0])
    draw_h_arrow(ax, 26.5, 31.0, 84.6, text=r"$\mathbf{a}, \boldsymbol{\omega}$", text_offset_y=1.4, fs=5.6)
    draw_h_arrow(ax, 26.5, 31.0, 63.8, text=r"$\mathbf{z}_{\mathrm{aid}}$", text_offset_y=1.4, fs=5.6)

    # ==========================================================================
    # CONTAINER 3: Martingale FDI & Isolation [x: 68.0 -> 98.5, y: 51.5 -> 98.5]
    # ==========================================================================
    draw_container(ax, 68.0, 51.5, 30.5, 47.0, "3. Martingale FDI & Isolation")

    # Top Formulas & Betting Principle: ALL BLACK TEXT
    ax.text(83.25, 90.8, "Game-Theoretic Online Betting", ha='center', va='center', fontsize=6.5, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(83.25, 87.2, r"Conformal $p_k = \frac{\sum \mathbb{I}(s_i > s_k) + U_k}{|\mathcal{H}_k| + 1} \in [0, 1]$", ha='center', va='center', fontsize=5.6, color=TEXT_BLACK, zorder=3)
    ax.text(83.25, 83.6, r"Payoff e-value:  $e_k = 1 + \epsilon_{\mathrm{bet}} (1 - 2 p_k)$", ha='center', va='center', fontsize=5.6, color=TEXT_BLACK, zorder=3)
    ax.text(83.25, 80.2, r"Wealth Martingale:  $M_k = \prod_{i=1}^k e_i$", ha='center', va='center', fontsize=6.0, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # Visual Quarantine Isolation Switch / Circuit Breaker (Colors on contacts)
    sw_x, sw_y = 74.5, 75.8
    ax.plot([sw_x - 1.2, sw_x], [sw_y, sw_y], color='#222222', linewidth=1.3, zorder=4)
    ax.plot([sw_x, sw_x + 1.2], [sw_y, sw_y + 1.3], color=FIG_COLOR_DARK_RED, linewidth=1.5, zorder=5)
    ax.plot([sw_x + 1.2, sw_x + 2.4], [sw_y, sw_y], color='#222222', linewidth=1.3, zorder=4)
    ax.scatter([sw_x, sw_x + 1.2], [sw_y, sw_y], color=FIG_COLOR_DARK_RED, s=10, zorder=6)
    ax.text(sw_x + 1.2, sw_y + 2.0, "Circuit Breaker", ha='center', va='center', fontsize=5.2, fontweight='bold', color=TEXT_BLACK)
    # Status Pill next to switch: Black text, white box, thin dark border
    ax.text(86.5, 75.8, "OPEN: ISOLATE GNSS", ha='center', va='center', fontsize=5.2, fontweight='bold',
            color=TEXT_BLACK, bbox=dict(boxstyle="round,pad=0.18", facecolor='#FFFFFF', edgecolor='#444444', linewidth=0.7), zorder=4)

    # Inset Axes for Wealth Martingale: Generous left margin (x=76.5 vs container border at x=68.0 = 8.5 units padding!)
    ax_mart = fig.add_axes([0.765, 0.535, 0.205, 0.165])
    np.random.seed(42)
    t_steps = np.linspace(0, 50, 100)
    wealth_h0 = np.cumprod(1.0 + 0.16 * (1.0 - 2.0 * np.random.uniform(0.35, 0.65, 100)))
    wealth_h1 = np.ones(100)
    wealth_h1[:60] = wealth_h0[:60]
    for k in range(60, 100):
        p_spoof = np.random.uniform(0.01, 0.08)
        wealth_h1[k] = wealth_h1[k-1] * (1.0 + 0.16 * (1.0 - 2.0 * p_spoof))

    ax_mart.plot(t_steps, np.log10(np.maximum(0.1, wealth_h0)), color=FIG_COLOR_GREEN, linewidth=1.3, label=r'$H_0$ Safe ($< 25$)')
    ax_mart.plot(t_steps, np.log10(np.maximum(0.1, wealth_h1)), color=FIG_COLOR_DARK_RED, linewidth=1.6, label=r'$H_1$ Spoofed $\log_{10}(M_k)$')
    ax_mart.axhline(2.0, color=FIG_COLOR_PURPLE, linestyle=':', linewidth=1.1, label=r'Ville: $1/\alpha=100$')
    ax_mart.axvline(30.0, color=FIG_COLOR_ORANGE, linestyle='--', linewidth=1.0)
    ax_mart.text(34.5, 2.5, "Quarantine\nTTD=4.05s", fontsize=5.0, color=TEXT_BLACK, fontweight='bold')
    ax_mart.set_xlim([0, 50])
    ax_mart.set_ylim([-0.5, 3.2])
    ax_mart.set_xlabel("Flight Steps $k$", labelpad=1, fontsize=5.8, color=TEXT_BLACK)
    ax_mart.set_ylabel(r"$\log_{10}(M_k)$", labelpad=1, fontsize=5.8, color=TEXT_BLACK)
    ax_mart.tick_params(labelsize=5.4, pad=1.5, colors=TEXT_BLACK)
    ax_mart.legend(loc='upper left', framealpha=0.9, fontsize=4.6, borderpad=0.2, labelspacing=0.2)
    ax_mart.grid(True)

    # Connections from Container 2 to Container 3 (Generous 4.0 gap: x in [64.0, 68.0])
    draw_h_arrow(ax, 64.0, 68.0, 84.6, text=r"$s_k$", text_offset_y=1.4, fs=6.0)

    # ==========================================================================
    # CONTAINER 4: Adaptive Conformal Filter (ACI-R) [x: 1.5 -> 47.5, y: 1.5 -> 45.5]
    # ==========================================================================
    draw_container(ax, 1.5, 1.5, 46.0, 44.0, "4. Adaptive ACI-R Filtering")

    # Formula Box (White bg, neutral border)
    box_acir_formula = patches.FancyBboxPatch((2.5, 25.0), 44.0, 15.0, boxstyle="round,pad=0.1,rounding_size=0.8",
                                             facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.9, zorder=2)
    ax.add_patch(box_acir_formula)
    ax.text(24.5, 37.8, "Closed-Form ACI-R Covariance Inflation Function", ha='center', va='center', fontsize=6.8, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(24.5, 33.0, r"$\phi_k = 1.0\ (s_k \leq \hat{q}_{10\%}) \quad \mid \quad 1.0 + \kappa \left( \frac{s_k - \hat{q}_{10\%}}{\hat{q}_{1\%} - \hat{q}_{10\%}} \right)^2 \quad \mid \quad \infty\ (\mathrm{Reject})$", ha='center', va='center', fontsize=5.8, color=TEXT_BLACK, zorder=3)
    ax.text(24.5, 28.0, r"Inflated:  $\tilde{\mathbf{R}}_k = \phi_k \mathbf{R}_k \quad \Rightarrow \quad \mathbf{K}_k(\phi_k) \to \mathbf{0}\ \mathrm{as}\ \|\boldsymbol{\nu}_k\| \to \infty$", ha='center', va='center', fontsize=5.8, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # Visual Annotation: 3 Non-Overlapping Zone Badges (White bg, thin neutral border, BLACK TEXT)
    ax.text(9.5, 21.8, "Zone I: Nominal\n$s \\leq \\hat{q}_{10}$", ha='center', va='center', fontsize=5.2, color=TEXT_BLACK, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.16", facecolor='#FFFFFF', edgecolor='#666666', linewidth=0.6), zorder=4)
    ax.text(24.5, 21.8, "Zone II: Inflation\n$\\hat{q}_{10} < s \\leq \\hat{q}_1$", ha='center', va='center', fontsize=5.2, color=TEXT_BLACK, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.16", facecolor='#FFFFFF', edgecolor='#666666', linewidth=0.6), zorder=4)
    ax.text(39.5, 21.8, "Zone III: Reject\n$s > \\hat{q}_1 \\Rightarrow \\phi = \\infty$", ha='center', va='center', fontsize=5.2, color=TEXT_BLACK, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.16", facecolor='#FFFFFF', edgecolor='#666666', linewidth=0.6), zorder=4)

    # Inset Axes for ACI-R Influence Curve (Padded with 8.0 margin on left so y-label never touches border!)
    ax_acir = fig.add_axes([0.095, 0.048, 0.365, 0.138])
    s_grid = np.linspace(0, 6, 200)
    psi_lin = s_grid
    psi_hub = np.where(s_grid <= 1.345, s_grid, 1.345)
    phi_grid = np.ones_like(s_grid)
    mask = (s_grid > 2.0) & (s_grid <= 3.8)
    phi_grid[mask] = 1.0 + 8.0 * ((s_grid[mask] - 2.0) / 1.8)**2
    phi_grid[s_grid > 3.8] = 1e6
    psi_acir = s_grid / phi_grid

    # Colored zone backgrounds directly on plot
    ax_acir.axvspan(0.0, 2.0, color=FIG_COLOR_GREEN, alpha=0.08, zorder=0)
    ax_acir.axvspan(2.0, 3.8, color=FIG_COLOR_ORANGE, alpha=0.10, zorder=0)
    ax_acir.axvspan(3.8, 6.0, color=FIG_COLOR_DARK_RED, alpha=0.08, zorder=0)

    ax_acir.plot(s_grid, psi_lin, color=FIG_COLOR_GREY, linestyle=':', linewidth=1.3, label=r'Linear $\chi^2$ (Unbounded)')
    ax_acir.plot(s_grid, psi_hub, color=FIG_COLOR_ORANGE, linestyle='--', linewidth=1.4, label='Huber (Saturated)')
    ax_acir.plot(s_grid, psi_acir, color=FIG_COLOR_GREEN, linewidth=1.8, label='Proposed ACI-R')
    ax_acir.axvline(3.8, color=FIG_COLOR_DARK_RED, linestyle='-.', linewidth=1.0, label='Glitch Cutoff')
    ax_acir.set_xlim([0, 6.0])
    ax_acir.set_ylim([0, 3.5])
    ax_acir.set_xlabel("Whitened Innovation $s_k$", labelpad=1, fontsize=6.0, color=TEXT_BLACK)
    ax_acir.set_ylabel(r"Influence $\psi(s)$", labelpad=1, fontsize=6.0, color=TEXT_BLACK)
    ax_acir.tick_params(labelsize=5.5, pad=1.5, colors=TEXT_BLACK)
    ax_acir.legend(loc='upper left', framealpha=0.9, fontsize=4.8)
    ax_acir.grid(True)

    # ==========================================================================
    # CONTAINER 5: Certified Pose Tubes & Reachability [x: 52.5 -> 98.5, y: 1.5 -> 45.5]
    # ==========================================================================
    draw_container(ax, 52.5, 1.5, 46.0, 44.0, "5. Certified Pose Reachability")

    # Formula Box (White bg, neutral border)
    box_tube_formula = patches.FancyBboxPatch((53.5, 23.5), 44.0, 16.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                             facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.9, zorder=2)
    ax.add_patch(box_tube_formula)
    ax.text(75.5, 37.8, "Non-Asymptotic Conformal Bounding Ellipsoids", ha='center', va='center', fontsize=6.8, fontweight='bold', color=TEXT_BLACK, zorder=3)
    ax.text(75.5, 33.2, r"Certified Tube:  $\mathcal{C}_k = \{ \mathbf{p} \in \mathbb{R}^3 : (\mathbf{p} - \hat{\mathbf{p}}_k)^T \mathbf{P}_{p,k}^{-1} (\mathbf{p} - \hat{\mathbf{p}}_k) \leq \rho_k^2 \}$", ha='center', va='center', fontsize=5.8, color=TEXT_BLACK, zorder=3)
    ax.text(75.5, 28.5, r"Guaranteed Coverage:  $\mathbb{P}(\mathbf{p}_k^{\mathrm{true}} \in \mathcal{C}_k) \geq 1 - \alpha_{\mathrm{pose}}$  (Empirical: 94.7%)", ha='center', va='center', fontsize=5.8, fontweight='bold', color=TEXT_BLACK, zorder=3)

    # DUAL VISUALIZATION IN CONTAINER 5:
    # 1. Left Graphic: 2D Safety Reachability Corridor & Geo-Fence (Generous padding!)
    box_reach = patches.FancyBboxPatch((53.5, 3.5), 20.5, 18.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                        facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.8, zorder=2)
    ax.add_patch(box_reach)
    ax.text(63.75, 20.6, "Safety Reachability Corridor", ha='center', va='center', fontsize=5.6, fontweight='bold', color=TEXT_BLACK, zorder=6)

    # Corridor tube band
    corr_x = np.linspace(54.5, 69.5, 30)
    corr_y_nom = 12.5 + 1.1 * np.sin((corr_x - 54.5) / 15.0 * np.pi)
    corr_y_upper = corr_y_nom + 2.5
    corr_y_lower = corr_y_nom - 2.5
    ax.fill_between(corr_x, corr_y_lower, corr_y_upper, color='#EBF5FB', alpha=0.85, zorder=3)
    ax.plot(corr_x, corr_y_upper, color=FIG_COLOR_BLUE, linestyle='--', linewidth=1.0, zorder=4)
    ax.plot(corr_x, corr_y_lower, color=FIG_COLOR_BLUE, linestyle='--', linewidth=1.0, zorder=4)
    ax.plot(corr_x, corr_y_nom, color=FIG_COLOR_GREEN, linewidth=1.5, zorder=5)

    # Drone on path
    ax.scatter([61.0], [12.5 + 1.1 * np.sin((61.0 - 54.5) / 15.0 * np.pi)], color=FIG_COLOR_GREEN, s=25, zorder=6)

    # Obstacle / Geo-fence boundary
    obs_x = [67.2, 72.0, 72.0, 67.2]
    obs_y = [15.4, 15.4, 18.8, 18.8]
    ax.add_patch(patches.Polygon(list(zip(obs_x, obs_y)), closed=True, facecolor='#FBE8EB', edgecolor=FIG_COLOR_DARK_RED, linewidth=1.1, hatch='//', zorder=4))
    ax.text(69.6, 17.1, "Hazard\nZone", ha='center', va='center', fontsize=4.6, fontweight='bold', color=FIG_COLOR_DARK_RED,
            bbox=dict(boxstyle="round,pad=0.12", facecolor='#FFFFFF', edgecolor='none', alpha=0.88), zorder=5)

    # Explanatory formula
    ax.text(63.75, 5.2, r"$\mathcal{C}_k \cap \mathcal{O} = \emptyset \Rightarrow \text{Guaranteed Safe}$", ha='center', va='center', fontsize=5.2, fontweight='bold', color=TEXT_BLACK, zorder=6)

    # 2. Right Graphic: 3D Conformal Ellipsoid Wireframe
    box_3d = patches.FancyBboxPatch((75.5, 3.5), 22.0, 18.5, boxstyle="round,pad=0.1,rounding_size=0.8",
                                     facecolor=BOX_BG, edgecolor=BOX_BORDER_LIGHT, linewidth=0.8, zorder=2)
    ax.add_patch(box_3d)

    ax_tube = fig.add_axes([0.768, 0.046, 0.200, 0.152], projection='3d')
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 10)
    rx, ry, rz = 1.2, 0.8, 0.5
    x_ell = rx * np.outer(np.cos(u), np.sin(v))
    y_ell = ry * np.outer(np.sin(u), np.sin(v))
    z_ell = rz * np.outer(np.ones_like(u), np.cos(v))

    ax_tube.plot_wireframe(x_ell, y_ell, z_ell, color=FIG_COLOR_PURPLE, alpha=0.35, linewidth=0.7)
    ax_tube.scatter([0], [0], [0], color=FIG_COLOR_GREEN, s=22, label=r'Pose $\hat{\mathbf{p}}_k$')
    ax_tube.scatter([0.2], [-0.15], [0.1], color='#111111', marker='x', s=30, label=r'GT $\mathbf{p}_k$')
    ax_tube.set_xlim([-1.5, 1.5])
    ax_tube.set_ylim([-1.5, 1.5])
    ax_tube.set_zlim([-1.0, 1.0])
    ax_tube.set_xticks([-1, 0, 1])
    ax_tube.set_yticks([-1, 0, 1])
    ax_tube.set_zticks([-1, 0, 1])
    ax_tube.set_xlabel('X', labelpad=-10, fontsize=5.0, color=TEXT_BLACK)
    ax_tube.set_ylabel('Y', labelpad=-10, fontsize=5.0, color=TEXT_BLACK)
    ax_tube.set_zlabel('Z', labelpad=-10, fontsize=5.0, color=TEXT_BLACK)
    ax_tube.tick_params(labelsize=4.4, pad=-3, colors=TEXT_BLACK)
    ax_tube.view_init(elev=22, azim=42)
    ax_tube.legend(loc='upper right', framealpha=0.95, fontsize=4.6)

    # ==========================================================================
    # INTER-CONTAINER ARROWS (Routed in open corridors)
    # ==========================================================================
    # 1. Whitening s_k score down to Container 4 (diagonal in 6.0 vertical gap)
    draw_arrow(ax, 47.5, 51.5, 24.5, 45.5, text=r"$s_k$ Score", text_pos=0.5, rad=0.06, fs=5.6)
    # 2. Martingale Quarantine Trigger signal to Filter / Isolation
    draw_arrow(ax, 74.0, 51.5, 41.5, 45.5, text="Quarantine Trigger", text_pos=0.48, rad=-0.06, fs=5.6)
    # 3. Filter estimate to Certified Pose Tube (Generous 5.0 gap: x in [47.5, 52.5])
    draw_h_arrow(ax, 47.5, 52.5, 31.5, text=r"$\hat{\mathbf{p}}_k, \mathbf{P}_k$", text_offset_y=1.4, fs=5.8)

    # Save to destination directories
    pdf_path = os.path.join(PLOTS_DIR, "fig1_coneskf_architecture.pdf")
    png_path = os.path.join(PLOTS_DIR, "fig1_coneskf_architecture.png")
    art_pdf = os.path.join(ARTIFACT_DIR, "fig1_coneskf_architecture.pdf")
    art_png = os.path.join(ARTIFACT_DIR, "fig1_coneskf_architecture.png")

    plt.savefig(pdf_path, bbox_inches='tight', pad_inches=0.15)
    plt.savefig(png_path, bbox_inches='tight', pad_inches=0.15)
    plt.savefig(art_pdf, bbox_inches='tight', pad_inches=0.15)
    plt.savefig(art_png, bbox_inches='tight', pad_inches=0.15)
    plt.close()
    print(f"  Successfully Generated: {pdf_path}")
    print(f"  Synced to Artifact Directory: {art_pdf}")

if __name__ == "__main__":
    create_rich_figure1()
