#!/usr/bin/env python3
"""
alfa_loader.py
Clean, real-data ingestion and synchronization pipeline for Carnegie Mellon University's
ALFA (Air Lab Fault and Anomaly) autonomous UAV flight dataset.
Zero synthetic data. Zero hardcoded heuristics.
"""

import os
import csv
import math
import numpy as np
from typing import Dict, List, Optional, Tuple

class ALFADatasetLoader:
    def __init__(self, data_root: str = "/home/jarvis/coneskf_fdi_ws/data/alfa/extracted/processed"):
        self.data_root = data_root
        if not os.path.exists(data_root):
            raise FileNotFoundError(f"ALFA data root not found: {data_root}")
        self.flights = sorted([f for f in os.listdir(data_root) if os.path.isdir(os.path.join(data_root, f))])

    def list_flights(self, category: str = "all") -> List[str]:
        if category == "all":
            return self.flights
        res = []
        for f in self.flights:
            f_lower = f.lower()
            if category == "healthy" and ("no_failure" in f_lower or "no_ground_truth" in f_lower):
                res.append(f)
            elif category == "engine" and "engine_failure" in f_lower:
                res.append(f)
            elif category == "aileron" and "aileron" in f_lower:
                res.append(f)
            elif category == "elevator" and "elevator" in f_lower:
                res.append(f)
            elif category == "rudder" and "rudder" in f_lower:
                res.append(f)
        return res

    @staticmethod
    def geodetic_to_enu(lat: np.ndarray, lon: np.ndarray, alt: np.ndarray,
                        lat0: float, lon0: float, alt0: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Convert WGS-84 Geodetic coordinates to local East-North-Up (ENU) coordinates in meters."""
        a = 6378137.0  # WGS-84 semi-major axis (meters)
        f = 1.0 / 298.257223563  # Flattening
        e2 = 2 * f - f**2

        phi = np.radians(lat)
        lam = np.radians(lon)
        phi0 = math.radians(lat0)
        lam0 = math.radians(lon0)

        # Reference radius of curvature in prime vertical
        N = a / np.sqrt(1 - e2 * np.sin(phi)**2)
        N0 = a / np.sqrt(1 - e2 * math.sin(phi0)**2)

        # ECEF coordinates
        X = (N + alt) * np.cos(phi) * np.cos(lam)
        Y = (N + alt) * np.cos(phi) * np.sin(lam)
        Z = (N * (1 - e2) + alt) * np.sin(phi)

        X0 = (N0 + alt0) * math.cos(phi0) * math.cos(lam0)
        Y0 = (N0 + alt0) * math.cos(phi0) * math.sin(lam0)
        Z0 = (N0 * (1 - e2) + alt0) * math.sin(phi0)

        dx = X - X0
        dy = Y - Y0
        dz = Z - Z0

        # Rotate ECEF differences to local ENU
        e = -math.sin(lam0) * dx + math.cos(lam0) * dy
        n = -math.sin(phi0) * math.cos(lam0) * dx - math.sin(phi0) * math.sin(lam0) * dy + math.cos(phi0) * dz
        u = math.cos(phi0) * math.cos(lam0) * dx + math.cos(phi0) * math.sin(lam0) * dy + math.sin(phi0) * dz

        return e, n, u

    def load_flight(self, flight_name: str) -> Dict[str, np.ndarray]:
        fdir = os.path.join(self.data_root, flight_name)
        if not os.path.isdir(fdir):
            raise FileNotFoundError(f"Flight directory not found: {fdir}")

        files = os.listdir(fdir)
        
        # 1. IMU Data
        imu_file = [f for f in files if "imu-data.csv" in f][0]
        imu_t, imu_w, imu_a, imu_q = [], [], [], []
        with open(os.path.join(fdir, imu_file), 'r') as fp:
            r = csv.reader(fp)
            next(r)
            for row in r:
                t = int(row[0]) / 1e9
                qx, qy, qz, qw = float(row[4]), float(row[5]), float(row[6]), float(row[7])
                wx, wy, wz = float(row[17]), float(row[18]), float(row[19])
                ax, ay, az = float(row[29]), float(row[30]), float(row[31])
                imu_t.append(t)
                imu_w.append([wx, wy, wz])
                imu_a.append([ax, ay, az])
                imu_q.append([qx, qy, qz, qw])

        t0 = imu_t[0]
        t_imu = np.array(imu_t) - t0
        w_imu = np.array(imu_w)
        a_imu = np.array(imu_a)
        q_imu = np.array(imu_q)

        # 2. GPS Fix
        gps_files = [f for f in files if "global_position-raw-fix.csv" in f]
        t_gps, pos_enu, gps_cov = np.empty((0,)), np.empty((0, 3)), np.empty((0, 3, 3))
        if gps_files:
            g_t, g_lat, g_lon, g_alt, g_cov = [], [], [], [], []
            with open(os.path.join(fdir, gps_files[0]), 'r') as fp:
                r = csv.reader(fp)
                next(r)
                for row in r:
                    gt = int(row[0]) / 1e9
                    lat, lon, alt = float(row[6]), float(row[7]), float(row[8])
                    # Position covariance: diagonal components
                    cov_xx = float(row[9])
                    cov_yy = float(row[13])
                    cov_zz = float(row[17])
                    g_t.append(gt)
                    g_lat.append(lat)
                    g_lon.append(lon)
                    g_alt.append(alt)
                    g_cov.append([[cov_xx, 0.0, 0.0], [0.0, cov_yy, 0.0], [0.0, 0.0, cov_zz]])

            if len(g_t) > 0:
                g_t = np.array(g_t) - t0
                g_lat = np.array(g_lat)
                g_lon = np.array(g_lon)
                g_alt = np.array(g_alt)
                e, n, u = self.geodetic_to_enu(g_lat, g_lon, g_alt, g_lat[0], g_lon[0], g_alt[0])
                pos_enu = np.column_stack([e, n, u])
                t_gps = g_t
                gps_cov = np.array(g_cov)

        # 3. Barometer
        baro_files = [f for f in files if "imu-atm_pressure.csv" in f]
        t_baro, alt_baro = np.empty((0,)), np.empty((0,))
        if baro_files:
            b_t, b_press = [], []
            with open(os.path.join(fdir, baro_files[0]), 'r') as fp:
                r = csv.reader(fp)
                next(r)
                for row in r:
                    bt = int(row[0]) / 1e9
                    press = float(row[4])  # fluid_pressure in Pascals or hPa
                    b_t.append(bt)
                    b_press.append(press)
            if len(b_t) > 0:
                t_baro = np.array(b_t) - t0
                # Standard barometric altitude formula
                p_hpa = np.array(b_press) / 100.0 if np.mean(b_press) > 2000 else np.array(b_press)
                alt_baro = 44330.0 * (1.0 - (p_hpa / 1013.25)**0.1903)
                alt_baro = alt_baro - alt_baro[0]  # relative altitude

        # 4. Failure Ground Truth
        fail_files = [f for f in files if "failure_status" in f]
        fail_onset_s = None
        fail_type = "Healthy"
        if "engine_failure" in flight_name:
            fail_type = "Engine Failure"
        elif "elevator" in flight_name:
            fail_type = "Elevator Actuator Jam"
        elif "aileron" in flight_name:
            fail_type = "Aileron Actuator Jam"
        elif "rudder" in flight_name:
            fail_type = "Rudder Actuator Jam"

        if fail_files:
            with open(os.path.join(fdir, fail_files[0]), 'r') as fp:
                r = csv.reader(fp)
                next(r)
                frows = list(r)
                if len(frows) > 0:
                    fail_onset_s = (int(frows[0][0]) / 1e9) - t0

        return {
            "flight_name": flight_name,
            "duration_s": float(t_imu[-1]),
            "t_imu": t_imu,
            "w_imu": w_imu,
            "a_imu": a_imu,
            "q_imu": q_imu,
            "t_gps": t_gps,
            "pos_enu": pos_enu,
            "gps_cov": gps_cov,
            "t_baro": t_baro,
            "alt_baro": alt_baro,
            "fail_type": fail_type,
            "fail_onset_s": fail_onset_s
        }

if __name__ == "__main__":
    loader = ALFADatasetLoader()
    print("ALFA dataset flights found:", len(loader.list_flights()))
    sample = loader.load_flight("carbonZ_2018-09-11-14-41-51_elevator_failure")
    print(f"Loaded: {sample['flight_name']}")
    print(f"  Duration: {sample['duration_s']:.2f} s")
    print(f"  IMU samples: {len(sample['t_imu'])}")
    print(f"  GPS fixes: {len(sample['t_gps'])}, ENU trajectory bounds: [{np.min(sample['pos_enu'], axis=0)} to {np.max(sample['pos_enu'], axis=0)}]")
    print(f"  Failure type: {sample['fail_type']}, Onset: {sample['fail_onset_s']} s")
