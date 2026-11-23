
import os, numpy as np

fdir = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed/carbonZ_2018-07-18-15-53-31_1_engine_failure"

# Load local_position-pose (position)
pose_file = f"{fdir}/carbonZ_2018-07-18-15-53-31_1_engine_failure-mavros-local_position-pose.csv"
pos_t, pos_x, pos_y, pos_z = [], [], [], []
with open(pose_file) as f:
    next(f)
    for line in f:
        p = line.strip().split(",")
        pos_t.append(int(p[0]) / 1e9)
        pos_x.append(float(p[4]))
        pos_y.append(float(p[5]))
        pos_z.append(float(p[6]))

t0 = pos_t[0]
pt = np.array(pos_t) - t0
P_mat = np.column_stack([pos_x, pos_y, pos_z])

# Velocity
vel_file = f"{fdir}/carbonZ_2018-07-18-15-53-31_1_engine_failure-mavros-local_position-velocity.csv"
vel_t, vel_x, vel_y, vel_z = [], [], [], []
with open(vel_file) as f:
    next(f)
    for line in f:
        p = line.strip().split(",")
        vel_t.append(int(p[0]) / 1e9)
        vel_x.append(float(p[4]))
        vel_y.append(float(p[5]))
        vel_z.append(float(p[6]))

vt = np.array(vel_t) - t0
V_mat = np.column_stack([vel_x, vel_y, vel_z])

onset = 116.17
print(f"Total pose points: {len(pt)}, duration: {pt[-1]:.2f}s, onset: {onset:.2f}s")

# Let's compute kinematic innovation residuals between constant-acceleration extrapolation and measured position
dt_step = np.diff(pt)
acc_est = np.diff(V_mat, axis=0) / dt_step[:, None]

# Innovation: discrepancy in expected position step
residuals = []
for i in range(1, len(pt)):
    dt = pt[i] - pt[i-1]
    if dt <= 0: continue
    p_pred = P_mat[i-1] + V_mat[i-1] * dt
    r = np.linalg.norm(P_mat[i] - p_pred) / (dt * np.linalg.norm(V_mat[i-1]) + 1e-4)
    residuals.append((pt[i], r))

# Conformal Betting Martingale
history = []
wealth = 1.0
eps = 0.20
threshold = 100.0
detected_t = None
max_wealth = 1.0
rng = np.random.default_rng(42)

for t, r in residuals:
    if len(history) >= 40:
        h = np.array(history)
        cg = np.sum(h > r)
        ce = np.sum(np.abs(h - r) < 1e-9)
        u = rng.uniform(0.0, 1.0)
        p_k = (cg + u * (ce + 1.0)) / (len(history) + 1.0)
        p_k = np.clip(p_k, 1e-6, 1.0 - 1e-6)

        e_k = 1.0 + eps * (1.0 - 2.0 * p_k)
        wealth *= e_k
        if wealth > max_wealth: max_wealth = wealth

        if wealth >= threshold and detected_t is None:
            detected_t = t

    if wealth < threshold:
        history.append(r)
        if len(history) > 100: history.pop(0)

print(f"Detection time: {detected_t}")
if detected_t:
    ttd = detected_t - onset
    print(f"Time-to-Detect (TTD): {ttd:.2f} seconds!")
