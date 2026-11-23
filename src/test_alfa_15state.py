
import numpy as np
import math, sys
sys.path.append("/home/jarvis/coneskf_fdi_ws/src")
from coneskf_datasets.alfa_loader import ALFADatasetLoader

def exp_so3(w):
    theta = np.linalg.norm(w)
    if theta < 1e-7:
        return np.eye(3)
    k = w / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

loader = ALFADatasetLoader()
data = loader.load_flight("carbonZ_2018-07-18-12-10-11_no_ground_truth")

t_imu = data["t_imu"]
w_imu = data["w_imu"]
a_imu = data["a_imu"]
t_gps = data["t_gps"]
pos_enu = data["pos_enu"]
gps_cov = data["gps_cov"]

print(f"Loaded flight: {len(t_imu)} IMU, {len(t_gps)} GPS points. Duration: {t_imu[-1]:.2f}s")

# Coarse gravity alignment
a_mean = np.mean(a_imu[:min(50, len(a_imu))], axis=0)
norm_a = np.linalg.norm(a_mean)
v_from = a_mean / norm_a
v_to = np.array([0, 0, 1.0])
v_cross = np.cross(v_from, v_to)
s_c = np.linalg.norm(v_cross)
c_c = np.dot(v_from, v_to)
R = np.eye(3)
if s_c > 1e-5:
    vx = skew(v_cross)
    R = np.eye(3) + vx + vx @ vx * ((1.0 - c_c) / (s_c**2))

p = pos_enu[0].copy()
v = np.zeros(3)
ba = np.zeros(3)
bg = np.zeros(3)
gravity = np.array([0, 0, -9.80665])

P = np.eye(15) * 0.1
P[6:9, 6:9] *= 0.01
P[9:12, 9:12] *= 0.001
P[12:15, 12:15] *= 0.0001

acc_noise = 0.08
gyr_noise = 0.004
acc_b_noise = 0.0004
gyr_b_noise = 0.00003

# Conformal martingale
history = []
wealth = 1.0
max_wealth = 1.0
eps = 0.16
min_cal = 60
win = 150
rng = np.random.default_rng(42)

gps_idx = 0
s_values = []

for i in range(1, len(t_imu)):
    dt = t_imu[i] - t_imu[i-1]
    if dt <= 0.0 or dt > 0.1: dt = 0.01

    acc = a_imu[i] - ba
    gyr = w_imu[i] - bg
    acc_w = R @ acc + gravity
    p += v * dt + 0.5 * acc_w * (dt**2)
    v += acc_w * dt
    dR = exp_so3(gyr * dt)
    R = R @ dR

    F = np.eye(15)
    F[0:3, 3:6] = np.eye(3) * dt
    F[0:3, 6:9] = -0.5 * R @ skew(acc) * (dt**2)
    F[0:3, 9:12] = -0.5 * R * (dt**2)
    F[3:6, 6:9] = -R @ skew(acc) * dt
    F[3:6, 9:12] = -R * dt
    F[6:9, 6:9] = exp_so3(-gyr * dt)
    F[6:9, 12:15] = -np.eye(3) * dt

    Qd = np.zeros((15, 15))
    Qd[0:3, 0:3] = np.eye(3) * (acc_noise**2 * dt**3 / 3.0)
    Qd[3:6, 3:6] = np.eye(3) * (acc_noise**2 * dt)
    Qd[6:9, 6:9] = np.eye(3) * (gyr_noise**2 * dt)
    Qd[9:12, 9:12] = np.eye(3) * (acc_b_noise**2 * dt)
    Qd[12:15, 12:15] = np.eye(3) * (gyr_b_noise**2 * dt)

    P = F @ P @ F.T + Qd
    P = 0.5 * (P + P.T)

    t_curr = t_imu[i]
    while gps_idx < len(t_gps) and t_gps[gps_idx] <= t_curr:
        z_pos = pos_enu[gps_idx]
        R_cov = gps_cov[gps_idx] if len(gps_cov) > gps_idx else np.eye(3) * 0.25
        # Ensure R_cov is well-conditioned
        if np.trace(R_cov) < 0.01: R_cov = np.eye(3) * 0.25

        nu = z_pos - p
        S = P[0:3, 0:3] + R_cov
        try:
            L = np.linalg.cholesky(S)
            w_nu = np.linalg.solve(L, nu)
            s_k = float(np.linalg.norm(w_nu))
        except np.linalg.LinAlgError:
            s_k = float(np.linalg.norm(nu)) / 0.5

        s_values.append(s_k)

        if len(history) >= min_cal:
            h = np.array(history)
            cg = np.sum(h > s_k)
            ce = np.sum(np.abs(h - s_k) < 1e-9)
            u = rng.uniform(0.0, 1.0)
            p_k = (cg + u * (ce + 1.0)) / (len(history) + 1.0)
            p_k = np.clip(p_k, 1e-6, 1.0 - 1e-6)
            e_k = 1.0 + eps * (1.0 - 2.0 * p_k)
            wealth *= e_k
            if wealth > max_wealth: max_wealth = wealth

        history.append(s_k)
        if len(history) > win: history.pop(0)

        # Full 15-state Joseph update
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)
        K = P[:, 0:3] @ np.linalg.inv(S)
        dx = K @ nu

        p += dx[0:3]
        v += dx[3:6]
        R = R @ exp_so3(dx[6:9])
        ba += dx[9:12]
        bg += dx[12:15]

        IKH = np.eye(15) - K @ H
        P = IKH @ P @ IKH.T + K @ R_cov @ K.T
        P = 0.5 * (P + P.T)

        gps_idx += 1

print(f"Healthy flight completed!")
print(f"Mean s_k = {np.mean(s_values):.3f}, Min = {np.min(s_values):.3f}, Max = {np.max(s_values):.3f}")
print(f"Max Wealth: {max_wealth:.3f} (Threshold = 100.0)")
print(f"False Alarm: {max_wealth >= 100.0}")
