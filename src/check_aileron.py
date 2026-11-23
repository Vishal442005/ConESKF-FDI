
import os, numpy as np

fdir = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed/carbonZ_2018-09-11-14-52-54_left_aileron__right_aileron__failure"

# Check roll angle and roll error
roll_file = [f for f in os.listdir(fdir) if "nav_info-roll.csv" in f][0]
roll_t, roll_val = [], []
with open(os.path.join(fdir, roll_file)) as f:
    next(f)
    for line in f:
        p = line.strip().split(",")
        roll_t.append(int(p[0]) / 1e9)
        roll_val.append(float(p[4]))

t0 = roll_t[0]
rt = np.array(roll_t) - t0
rval = np.array(roll_val)

onset = 176.68
print(f"Aileron Jam Onset: {onset:.2f} s, Duration: {rt[-1]:.2f} s")
print("Roll behavior around onset:")
for i in range(len(rt)):
    if rt[i] >= onset - 5.0 and rt[i] <= onset + 15.0 and i % 5 == 0:
        print(f"t={rt[i]:.2f}s: roll={rval[i]:.2f} rad ({np.degrees(rval[i]):.1f} deg)")
