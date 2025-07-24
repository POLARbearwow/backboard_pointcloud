#!/usr/bin/env python3
"""
Convert PointCloud2 messages in a ROS2 bag to PCD (.pcd) files (binary).

Parameters are hard-coded at top of script for convenience.
Each PointCloud2 message on the target topic is written as one PCD file
named 000000.pcd, 000001.pcd, ... in OUTPUT_DIR.

PCD header follows format version 0.7 with fields x y z intensity (float32).
"""
from pathlib import Path
import struct
import sys
import numpy as np

import rclpy.serialization
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# ----- USER CONFIGURABLE PARAMS -----
BAG_PATH = "/home/niu/Desktop/lidar_moving"  # directory containing ROS2 bag
OUTPUT_DIR = "/home/niu/Desktop/pointcloud_ws/pcd_out"  # where .pcd files are saved
TOPIC = "/livox/lidar"  # PointCloud2 topic
# ------------------------------------

FIELDS = ('x', 'y', 'z', 'intensity')


def msg_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 to Nx4 float32 numpy array."""
    plain_pts = []
    for p in pc2.read_points(msg, field_names=FIELDS, skip_nans=True):
        plain_pts.append([float(p[0]), float(p[1]), float(p[2]), float(p[3])])
    if not plain_pts:
        return np.empty((0, 4), dtype=np.float32)
    return np.asarray(plain_pts, dtype=np.float32)


def write_pcd(points: np.ndarray, filepath: Path):
    """Write Nx4 float32 points to binary PCD file."""
    n_points = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n_points}\n"
        "HEIGHT 1\n"
        f"POINTS {n_points}\n"
        "DATA binary\n"
    )
    with open(filepath, "wb") as f:
        f.write(header.encode('ascii'))
        # Ensure header ends with newline
        # Binary data: write flat bytes
        f.write(points.tobytes())


def open_reader(bag_path: str):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def main():
    bag_path = Path(BAG_PATH)
    if not bag_path.exists():
        sys.exit(f"Bag path {bag_path} does not exist")

    out_dir = Path(OUTPUT_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(str(bag_path))

    print(f"Converting bag {bag_path} -> {out_dir} for topic {TOPIC}")

    file_index = 0
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic != TOPIC:
            continue

        msg = rclpy.serialization.deserialize_message(raw, PointCloud2)
        pts = msg_to_numpy(msg)
        filename = out_dir / f"{file_index:06d}.pcd"
        write_pcd(pts, filename)
        print(f"Wrote {filename} ({pts.shape[0]} pts)")
        file_index += 1

    print("Done. Total files:", file_index)


if __name__ == "__main__":
    main() 