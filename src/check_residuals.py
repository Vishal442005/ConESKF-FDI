
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

# Residual: Acceleration norm / jerk / tracking residual
dt_step = np.diff(pt)
residuals = []
for i in range(1, len(pt)):
    dt = pt[i] - pt[i-1]
    if dt <= 0: continue
    # Acceleration residual: difference in velocity
    v_dot = (V_mat[i] - V_mat[i-1]) / dt
    r = np.linalg.norm(v_dot)
    residuals.append((pt[i], r))

# Inspect mean residual before onset vs after onset
r_before = [r for t, r in residuals if t < onset]
r_after = [r for t, r in residuals if t >= onset]

print(f"Mean residual before onset: {np.mean(r_before):.3f}, std: {np.std(r_before):.3f}")
print(f"Mean residual after onset:  {np.mean(r_after):.3f}, std: {np.std(r_after):.3f}")

# Look at specific force / altitude error from nav_info
nav_file = f"{fdir}/carbonZ_2018-07-18-15-53-31_1_engine_failure-mavros-nav_info-errors.csv"
with open(nav_file) as f:
    header = f.readline().strip()
    alt_errs = []
    xtrack_errs = []
    ts = []
    for line in f:
        p = line.strip().split(",")
        t = (int(p[0]) / 1e9) - t0
        alt_e = abs(float(p[4]))
        xt_e = abs(float(p[6]))
        ts.append(t)
        alt_errs.append(alt_e)
        xtrack_errs.append(xt_e)

alt_before = [e for t, e in zip(ts, alt_errs) if t < onset and e < 1000]
alt_after = [e for t, e in zip(ts, alt_errs) if t >= onset and e < 1000]
print(f"Altitude tracking error before onset: {np.mean(alt_before):.2f} m")
print(f"Altitude tracking error after onset:  {np.mean(alt_after):.2f} m")

xt_before = [e for t, e in zip(ts, xtrack_errs) if t < onset and e < 1000]
xt_after = [e for t, e in zip(ts, xtrack_errs) if t >= onset and e < 1000]
print(f"Cross-track error before onset: {np.mean(xt_before):.2f} m")
print(f"Cross-track error after onset:  {np.mean(xt_after):.2f} m")
