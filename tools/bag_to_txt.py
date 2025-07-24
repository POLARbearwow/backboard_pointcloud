#!/usr/bin/env python3
"""
Convert PointCloud2 messages in a ROS2 bag to plain text files.

Each TXT file contains concatenated points from `GROUP` consecutive frames.
Format per line:  x y z intensity   (space-separated, float values)
File naming: 000000.txt, 000001.txt, ...

Parameters are fixed below; edit to suit your workspace.
"""
from pathlib import Path
import sys
import numpy as np
import rclpy.serialization
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# ---------- USER CONFIG ----------
BAG_PATH = "/home/niu/Desktop/lidar_moving"          # directory containing ROS2 bag
OUTPUT_DIR = "/home/niu/Desktop/pointcloud_ws/txt_pc"  # directory to save txt files
TOPIC = "/livox/lidar"                                # PointCloud2 topic
GROUP = 3                                              # frames per txt
# ---------------------------------

# Fields to export
FIELDS = ("x", "y", "z", "intensity")


def open_reader(bag_dir: str):
    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def msg_to_numpy(msg: PointCloud2) -> np.ndarray:
    plain_pts = []
    for p in pc2.read_points(msg, field_names=FIELDS, skip_nans=True):
        plain_pts.append([float(p[0]), float(p[1]), float(p[2]), float(p[3])])

    if not plain_pts:
        return np.empty((0, 4), dtype=np.float32)

    return np.asarray(plain_pts, dtype=np.float32)


def write_txt(out_path: Path, pts: np.ndarray):
    np.savetxt(out_path, pts, fmt='%.6f', delimiter=',')


def main():
    bag_path = Path(BAG_PATH)
    if not bag_path.exists():
        sys.exit(f"Bag path {bag_path} not found")

    out_dir = Path(OUTPUT_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(str(bag_path))
    buffer = []
    index = 0

    print(f"Reading {bag_path} topic {TOPIC}, grouping {GROUP} frames per txt ...")

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != TOPIC:
            continue
        msg = rclpy.serialization.deserialize_message(raw, PointCloud2)
        cloud_np = msg_to_numpy(msg)
        buffer.append(cloud_np)
        if len(buffer) == GROUP:
            merged = np.concatenate(buffer, axis=0)
            filename = out_dir / f"{index:06d}.txt"
            write_txt(filename, merged)
            print(f"Wrote {filename} ({merged.shape[0]} pts from {GROUP} frames)")
            index += 1
            buffer.clear()

    print("Done. TXT files saved in", out_dir)


if __name__ == "__main__":
    main() 