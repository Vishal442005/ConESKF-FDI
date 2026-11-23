
import os, numpy as np

fdir = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed/carbonZ_2018-07-18-15-53-31_1_engine_failure"

# Load local_position-velocity
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

t0 = vel_t[0]
vt = np.array(vel_t) - t0
vx = np.array(vel_x)
vy = np.array(vel_y)
vz = np.array(vel_z)
speed = np.sqrt(vx**2 + vy**2 + vz**2)

onset = 116.17
print(f"Engine Cutout Onset: {onset:.2f} s")
print("Telemetry around onset:")
for i in range(len(vt)):
    if vt[i] >= onset - 5.0 and vt[i] <= onset + 15.0 and i % 3 == 0:
        print(f"t={vt[i]:.2f}s: speed={speed[i]:.2f} m/s, vz={vz[i]:.2f} m/s (downward), vx={vx[i]:.2f}, vy={vy[i]:.2f}")
