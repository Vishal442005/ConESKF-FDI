
import numpy as np

leica_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_leica.csv"
data = np.loadtxt(leica_csv, delimiter=',', skiprows=1)
t = data[:, 0]
p = data[:, 1:4]
dt = np.diff(t)
v = np.linalg.norm(np.diff(p, axis=0) / dt[:, None], axis=1)

for i in range(len(t)):
    if t[i] >= 35.0 and t[i] <= 55.0 and i % 10 == 0:
        print(f"t={t[i]:.2f}s: pos=[{p[i,0]:.2f}, {p[i,1]:.2f}, {p[i,2]:.2f}], v={v[min(i, len(v)-1)]:.3f} m/s")
