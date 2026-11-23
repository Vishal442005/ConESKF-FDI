
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

for i in range(len(rt)):
    if rt[i] >= 100.0 and rt[i] <= 112.0 and i % 15 == 0:
        print(f"t={rt[i]:.2f}s: roll={rpy_arr[i,0]:.2f}, pitch={rpy_arr[i,1]:.2f}, yaw={rpy_arr[i,2]:.2f}")
