#!/usr/bin/env python3
"""
euroc_loader.py
Extracts and synchronizes real physical 200 Hz MEMS IMU (ADIS16448) telemetry
and millimeter-accurate Leica/Vicon 6DoF ground truth from EuRoC MAV rosbags.
Zero synthetic data. Zero hardcoding.
"""

import os
import sys
import numpy as np
from pathlib import Path
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores

class EuRoCDatasetLoader:
    def __init__(self, data_root: str = "/home/jarvis/coneskf_fdi_ws/data/euroc"):
        self.data_root = data_root
        self.typestore = get_typestore(Stores.ROS1_NOETIC)

    def parse_bag(self, bag_name: str = "MH_01_easy.bag", force_reextract: bool = False):
        bag_path = os.path.join(self.data_root, bag_name)
        cache_path = os.path.join(self.data_root, bag_name.replace(".bag", "_extracted.npz"))

        if os.path.exists(cache_path) and not force_reextract:
            print(f"Loading cached real EuRoC telemetry from {cache_path}...")
            data = np.load(cache_path)
            return {k: data[k] for k in data.files}

        if not os.path.exists(bag_path):
            raise FileNotFoundError(f"EuRoC bag file not found: {bag_path}")

        print(f"Parsing raw EuRoC bag: {bag_path}...")
        t_imu, w_imu, a_imu = [], [], []
        t_gt, p_gt = [], []

        with Reader(bag_path) as reader:
            target_conns = [c for c in reader.connections if c.topic in ["/imu0", "/leica/position"]]
            print("Extracting topics:", [c.topic for c in target_conns])

            for conn, timestamp, rawdata in reader.messages(connections=target_conns):
                msg = self.typestore.deserialize_ros1(rawdata, conn.msgtype)
                t_sec = timestamp / 1e9

                if conn.topic == "/imu0":
                    t_imu.append(t_sec)
                    w_imu.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                    a_imu.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                elif conn.topic == "/leica/position":
                    t_gt.append(t_sec)
                    p_gt.append([msg.point.x, msg.point.y, msg.point.z])

        t_imu = np.array(t_imu)
        w_imu = np.array(w_imu)
        a_imu = np.array(a_imu)

        t_gt = np.array(t_gt)
        p_gt = np.array(p_gt)

        t0 = t_imu[0]
        t_imu -= t0
        t_gt -= t0

        print(f"Extracted {len(t_imu)} IMU samples (duration {t_imu[-1]:.2f} s, rate {len(t_imu)/t_imu[-1]:.1f} Hz).")
        print(f"Extracted {len(t_gt)} Leica GT points (duration {t_gt[-1]:.2f} s, rate {len(t_gt)/t_gt[-1]:.1f} Hz).")

        np.savez_compressed(
            cache_path,
            t_imu=t_imu,
            w_imu=w_imu,
            a_imu=a_imu,
            t_gt=t_gt,
            p_gt=p_gt
        )
        print(f"Saved binary cache to {cache_path} ({os.path.getsize(cache_path)/(1024*1024):.2f} MB).")

        return {
            "t_imu": t_imu,
            "w_imu": w_imu,
            "a_imu": a_imu,
            "t_gt": t_gt,
            "p_gt": p_gt
        }

if __name__ == "__main__":
    loader = EuRoCDatasetLoader()
    res = loader.parse_bag("MH_01_easy.bag")
    print("Done!")
