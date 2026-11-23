
import numpy as np

leica_csv = "/home/jarvis/coneskf_fdi_ws/data/euroc/MH_01_leica.csv"
data = np.loadtxt(leica_csv, delimiter=',', skiprows=1)
t = data[:, 0]
p = data[:, 1:4]

# Find where altitude p[:, 2] starts rising or velocity > 0.1 m/s
dt = np.diff(t)
v = np.linalg.norm(np.diff(p, axis=0) / dt[:, None], axis=1)

print(f"Total time range: {t[0]:.2f} to {t[-1]:.2f} s")
for i in range(0, min(500, len(t)), 25):
    print(f"t={t[i]:.2f}s: pos=[{p[i,0]:.2f}, {p[i,1]:.2f}, {p[i,2]:.2f}], v={v[min(i, len(v)-1)]:.3f} m/s")
