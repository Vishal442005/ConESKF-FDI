
import os

data_root = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed"
flights = sorted([f for f in os.listdir(data_root) if os.path.isdir(os.path.join(data_root, f))])

flight_info = []
for f in flights:
    fdir = os.path.join(data_root, f)
    files = os.listdir(fdir)
    imu_files = [x for x in files if "imu-data.csv" in x]
    if not imu_files:
        continue
    
    with open(os.path.join(fdir, imu_files[0])) as fp:
        next(fp)
        t0_imu = int(next(fp).split(",")[0]) / 1e9

    with open(os.path.join(fdir, imu_files[0])) as fp:
        for line in fp: pass
        t_end_imu = int(line.split(",")[0]) / 1e9
    
    duration = t_end_imu - t0_imu

    fail_files = [x for x in files if "failure_status" in x]
    fail_onset = None
    fail_type = "Healthy / Normal"

    if "engine_failure" in f.lower():
        fail_type = "Engine Cutout"
    elif "elevator" in f.lower():
        fail_type = "Elevator Actuator Jam"
    elif "aileron" in f.lower():
        fail_type = "Aileron Actuator Jam"
    elif "rudder" in f.lower():
        fail_type = "Rudder Actuator Jam"

    if fail_files:
        with open(os.path.join(fdir, fail_files[0])) as fp:
            next(fp)
            try:
                first_line = next(fp)
                t0_fail = int(first_line.split(",")[0]) / 1e9
                fail_onset = t0_fail - t0_imu
            except StopIteration:
                pass

    flight_info.append({
        "flight": f,
        "type": fail_type,
        "duration": duration,
        "onset": fail_onset
    })

healthy = [x for x in flight_info if x["type"] == "Healthy / Normal"]
failures = [x for x in flight_info if x["type"] != "Healthy / Normal"]
with_onset = [x for x in failures if x["onset"] is not None]

print(f"Total flights: {len(flight_info)}")
print(f"Healthy flights: {len(healthy)}")
print(f"Failure flights: {len(failures)}")
print(f"Failure flights with exact onset: {len(with_onset)} / {len(failures)}")
for x in with_onset[:10]:
    print(f"  {x['flight'][:40]} | {x['type']} | Onset: {x['onset']:.2f}s | Dur: {x['duration']:.2f}s")
