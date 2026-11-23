
import os

fdir = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed/carbonZ_2018-07-18-15-53-31_1_engine_failure"

# 1. IMU time
imu_file = os.path.join(fdir, "carbonZ_2018-07-18-15-53-31_1_engine_failure-mavros-imu-data.csv")
with open(imu_file) as f:
    next(f)
    first_imu_line = next(f)
    t0_imu = int(first_imu_line.split(",")[0]) / 1e9

# 2. Failure status time
fail_file = os.path.join(fdir, "carbonZ_2018-07-18-15-53-31_1_engine_failure-failure_status-engines.csv")
with open(fail_file) as f:
    next(f)
    first_fail_line = next(f)
    t0_fail = int(first_fail_line.split(",")[0]) / 1e9

print(f"IMU start time: {t0_imu:.3f}")
print(f"Fail status start time: {t0_fail:.3f}")
print(f"Difference (onset relative to flight start): {t0_fail - t0_imu:.3f} seconds!")
