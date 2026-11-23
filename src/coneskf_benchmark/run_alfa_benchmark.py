
import os, sys, math, time, csv, numpy as np

sys.path.append("/home/jarvis/coneskf_fdi_ws/src")
from coneskf_datasets.alfa_loader import ALFADatasetLoader

loader = ALFADatasetLoader()
flights = loader.list_flights()

class ConformalMartingale:
    def __init__(self, window_size=150, min_calib=40, eps=0.25, threshold=25.0):
        self.window_size = window_size
        self.min_calib = min_calib
        self.eps = eps
        self.threshold = threshold
        self.wealth = 1.0
        self.history = []
        self.is_quarantined = False
        self.rng = np.random.default_rng(42)

    def update(self, s_k):
        if self.is_quarantined:
            return self.wealth

        if len(self.history) < self.min_calib:
            self.history.append(s_k)
            return self.wealth

        h = np.array(self.history)
        cg = np.sum(h > s_k)
        ce = np.sum(np.abs(h - s_k) < 1e-9)
        u = self.rng.uniform(0.0, 1.0)
        p_k = (cg + u * (ce + 1.0)) / (len(self.history) + 1.0)
        p_k = np.clip(p_k, 1e-6, 1.0 - 1e-6)

        e_k = 1.0 + self.eps * (1.0 - 2.0 * p_k)
        self.wealth *= e_k

        if self.wealth >= self.threshold:
            self.is_quarantined = True
        else:
            self.history.append(s_k)
            if len(self.history) > self.window_size:
                self.history.pop(0)

        return self.wealth

def evaluate_flight(flight_name):
    data = loader.load_flight(flight_name)
    fdir = os.path.join(loader.data_root, flight_name)

    pose_files = [f for f in os.listdir(fdir) if "local_position-pose.csv" in f]
    vel_files = [f for f in os.listdir(fdir) if "local_position-velocity.csv" in f]

    if not pose_files or not vel_files:
        return None

    pt, pos_list = [], []
    with open(os.path.join(fdir, pose_files[0])) as fp:
        next(fp)
        for line in fp:
            p = line.strip().split(",")
            pt.append(int(p[0]) / 1e9)
            pos_list.append([float(p[4]), float(p[5]), float(p[6])])

    t0 = pt[0]
    pt = np.array(pt) - t0
    P_mat = np.array(pos_list)

    vt, vel_list = [], []
    with open(os.path.join(fdir, vel_files[0])) as fp:
        next(fp)
        for line in fp:
            p = line.strip().split(",")
            vt.append((int(p[0]) / 1e9) - t0)
            vel_list.append([float(p[4]), float(p[5]), float(p[6])])

    V_mat = np.array(vel_list)

    fail_type = data["fail_type"]
    fail_onset = data["fail_onset_s"]

    residuals = []
    v_idx = 0
    for i in range(1, len(pt)):
        dt = pt[i] - pt[i-1]
        if dt <= 0.001 or dt > 1.0: continue
        while v_idx < len(vt) and vt[v_idx] < pt[i-1]:
            v_idx += 1
        v_curr = V_mat[min(v_idx, len(V_mat)-1)]
        p_pred = P_mat[i-1] + v_curr * dt
        r = np.linalg.norm(P_mat[i] - p_pred) / (dt * max(1.0, np.linalg.norm(v_curr)))
        residuals.append((pt[i], r))

    mart = ConformalMartingale(window_size=150, min_calib=40, eps=0.25, threshold=25.0)

    t_start = time.perf_counter()
    detect_t = None
    max_w = 1.0

    for t_step, r_val in residuals:
        w = mart.update(r_val)
        if w > max_w: max_w = w
        if mart.is_quarantined and detect_t is None:
            detect_t = t_step

    t_elapsed = time.perf_counter() - t_start
    mean_lat_us = (t_elapsed / max(1, len(residuals))) * 1e6

    is_healthy = (fail_type == "Healthy")
    false_alarm = False
    ttd = None

    if is_healthy:
        false_alarm = (detect_t is not None)
    else:
        if fail_onset is not None:
            if detect_t is not None:
                if detect_t < fail_onset:
                    false_alarm = True
                else:
                    ttd = detect_t - fail_onset
            else:
                ttd = None

    return {
        "flight": flight_name,
        "type": fail_type,
        "duration_s": float(pt[-1]),
        "onset_s": fail_onset,
        "detect_s": detect_t,
        "ttd_s": ttd,
        "max_wealth": max_w,
        "false_alarm": false_alarm,
        "latency_us": mean_lat_us
    }

results = []
for f in flights:
    res = evaluate_flight(f)
    if res:
        results.append(res)
        onset_str = f"{res['onset_s']:.1f}s" if res['onset_s'] else "N/A"
        ttd_str = f"{res['ttd_s']:.2f}s" if res['ttd_s'] is not None else "None"
        print(f"[{res['type'][:12]:12}] {res['flight'][:35]:35} | Onset: {onset_str:6} | TTD: {ttd_str:7} | FA: {str(res['false_alarm']):5} | MaxW: {res['max_wealth']:.1f}")

healthy_res = [r for r in results if r["type"] == "Healthy"]
engine_res = [r for r in results if "Engine" in r["type"]]
aileron_res = [r for r in results if "Aileron" in r["type"]]
elevator_res = [r for r in results if "Elevator" in r["type"]]
rudder_res = [r for r in results if "Rudder" in r["type"]]

print("\n=======================================================")
print(f"Healthy Flights: {len(healthy_res)} | False Alarms: {sum(1 for r in healthy_res if r['false_alarm'])} / {len(healthy_res)}")
print(f"Engine Cutouts: {len(engine_res)} | Detected: {sum(1 for r in engine_res if r['ttd_s'] is not None)} / {len(engine_res)}")
if any(r['ttd_s'] is not None for r in engine_res):
    ttd_eng = [r['ttd_s'] for r in engine_res if r['ttd_s'] is not None]
    print(f"  Mean Engine TTD: {np.mean(ttd_eng):.2f} s")

print(f"Aileron Jams: {len(aileron_res)} | Detected: {sum(1 for r in aileron_res if r['ttd_s'] is not None)} / {len(aileron_res)}")
if any(r['ttd_s'] is not None for r in aileron_res):
    ttd_ail = [r['ttd_s'] for r in aileron_res if r['ttd_s'] is not None]
    print(f"  Mean Aileron TTD: {np.mean(ttd_ail):.2f} s")

print(f"Elevator Jams: {len(elevator_res)} | Detected: {sum(1 for r in elevator_res if r['ttd_s'] is not None)} / {len(elevator_res)}")
print(f"Rudder Jams: {len(rudder_res)} | Detected: {sum(1 for r in rudder_res if r['ttd_s'] is not None)} / {len(rudder_res)}")
print("=======================================================\n")

out_csv = "/home/jarvis/coneskf_fdi_ws/results/table3_alfa_benchmarks.csv"
with open(out_csv, "w", newline="") as fp:
    w = csv.writer(fp)
    w.writerow(["Flight", "Category", "Duration_s", "Onset_s", "Detected_s", "TTD_s", "MaxWealth", "FalseAlarm", "Latency_us"])
    for r in results:
        w.writerow([
            r["flight"],
            r["type"],
            f"{r['duration_s']:.2f}",
            f"{r['onset_s']:.2f}" if r["onset_s"] else "N/A",
            f"{r['detect_s']:.2f}" if r["detect_s"] else "None",
            f"{r['ttd_s']:.2f}" if r["ttd_s"] is not None else "None",
            f"{r['max_wealth']:.2f}",
            "TRUE" if r["false_alarm"] else "FALSE",
            f"{r['latency_us']:.2f}"
        ])
print(f"Table 3 successfully saved to: {out_csv}")
