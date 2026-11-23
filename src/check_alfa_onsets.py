
from coneskf_datasets.alfa_loader import ALFADatasetLoader
import json

loader = ALFADatasetLoader()
flights = loader.list_flights()
print(f"Total flights: {len(flights)}")

results = []
for f in flights:
    try:
        data = loader.load_flight(f)
        results.append({
            "flight": f,
            "type": data["fail_type"],
            "duration": data["duration_s"],
            "onset": data["fail_onset_s"],
            "imu_count": len(data["t_imu"]),
            "gps_count": len(data["t_gps"])
        })
    except Exception as e:
        print(f"Error loading {f}: {e}")

print(f"Successfully loaded {len(results)} flights.")
has_onset = [r for r in results if r["onset"] is not None]
print(f"Flights with explicit onset timestamp: {len(has_onset)}")
for r in has_onset[:10]:
    print(f"  {r['flight']}: Type={r['type']}, Onset={r['onset']:.2f}s, Dur={r['duration']:.2f}s")

no_onset_fails = [r for r in results if r["onset"] is None and r["type"] != "Healthy"]
print(f"Failure flights without explicit failure_status file: {len(no_onset_fails)}")
if len(no_onset_fails) > 0:
    for r in no_onset_fails[:5]:
        print(f"  {r['flight']}: Type={r['type']}")
