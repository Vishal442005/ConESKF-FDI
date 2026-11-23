
import os, numpy as np

fdir = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed/carbonZ_2018-09-11-14-52-54_left_aileron__right_aileron__failure"

rpy_file = [f for f in os.listdir(fdir) if "mavctrl-rpy.csv" in f][0]
rpy_t, rpy_vals = [], []
with open(os.path.join(fdir, rpy_file)) as fp:
    next(fp)
    for line in fp:
        p = line.strip().split(",")
        rpy_t.append(int(p[0]) / 1e9)
        rpy_vals.append([float(p[1]), float(p[2]), float(p[3])])

t0 = rpy_t[0]
rt = np.array(rpy_t) - t0
rpy_arr = np.array(rpy_vals)

onset = 105.2  # Ground truth onset
print(f"50 Hz RPY points: {len(rt)}, duration: {rt[-1]:.2f}s, onset: {onset:.2f}s")

dt_rpy = np.diff(rt)
drpy = np.diff(rpy_arr, axis=0) / np.maximum(1e-4, dt_rpy[:, None])
r_scores = np.linalg.norm(drpy, axis=1)

history = []
wealth = 1.0
eps = 0.20
threshold = 100.0
detected_t = None
max_wealth = 1.0
rng = np.random.default_rng(42)

for i in range(len(r_scores)):
    t_curr = rt[i+1]
    s_k = float(r_scores[i])

    if len(history) >= 100:
        h = np.array(history)
        cg = np.sum(h > s_k)
        ce = np.sum(np.abs(h - s_k) < 1e-9)
        u = rng.uniform(0.0, 1.0)
        p_k = (cg + u * (ce + 1.0)) / (len(history) + 1.0)
        p_k = np.clip(p_k, 1e-6, 1.0 - 1e-6)
        e_k = 1.0 + eps * (1.0 - 2.0 * p_k)
        wealth *= e_k
        if wealth > max_wealth: max_wealth = wealth

        if wealth >= threshold and detected_t is None:
            detected_t = t_curr

    if wealth < threshold:
        history.append(s_k)
        if len(history) > 250: history.pop(0)

print(f"Max Wealth: {max_wealth:.2f}")
print(f"Detected at: {detected_t:.2f} s" if detected_t else "Not detected")
if detected_t:
    print(f"Time-to-Detect (TTD): {detected_t - onset:.2f} seconds!")
