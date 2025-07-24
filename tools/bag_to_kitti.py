#!/usr/bin/env python3
"""
Convert PointCloud2 messages in a ROS2 bag to KITTI .bin files.

• Reads target topic (default /livox/lidar) from the bag.
• Accumulates every `--group` frames (default 3) into one KITTI file.
• KITTI binary format: consecutive float32 values [x y z intensity] per point.

Usage:
  python3 bag_to_kitti.py --bag /path/to/ros2_bag  --output-dir /path/to/save  \
          [--topic /points] [--group 3]

Requires ROS 2 environment (rclpy, rosbag2_py) and numpy.
"""
import sys
from pathlib import Path

import numpy as np

import rclpy.serialization
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# fields to extract
FIELDS = ('x', 'y', 'z', 'intensity')

# ---- USER CONFIGURABLE PARAMETERS ----
BAG_PATH = "/home/niu/Desktop/lidar_moving"  # directory containing ROS2 bag
OUTPUT_DIR = "/home/niu/Desktop/pointcloud_ws/kitti_pc"  # where .bin files are written
TOPIC = "/livox/lidar"  # target PointCloud2 topic
GROUP = 3  # number of frames per KITTI file
# --------------------------------------


def open_reader(bag_path: str):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def deserialize_cloud(raw: bytes) -> PointCloud2:
    return rclpy.serialization.deserialize_message(raw, PointCloud2)


def msg_to_numpy(msg: PointCloud2):
    # Use read_points; convert each structured element to plain list for clean dtype
    plain_pts = []
    for p in pc2.read_points(msg, field_names=FIELDS, skip_nans=True):
        # p is a numpy.void; convert to tuple/list to avoid structured dtype
        plain_pts.append([float(p[0]), float(p[1]), float(p[2]), float(p[3])])

    if not plain_pts:
        return np.empty((0, len(FIELDS)), dtype=np.float32)

    return np.asarray(plain_pts, dtype=np.float32)


def main():
    bag_path = Path(BAG_PATH)
    if not bag_path.exists():
        sys.exit(f"Bag path {bag_path} does not exist")

    out_dir = Path(OUTPUT_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(str(bag_path))

    buffer = []  # list of numpy arrays, one per frame
    kitti_index = 0

    print(f"Processing bag {bag_path} on topic {TOPIC} ...")

    while reader.has_next():
        topic, raw_data, t = reader.read_next()
        if topic != TOPIC:
            continue

        msg = deserialize_cloud(raw_data)
        np_pts = msg_to_numpy(msg)
        buffer.append(np_pts)

        if len(buffer) == GROUP:
            merged = np.concatenate(buffer, axis=0)
            filename = out_dir / f"{kitti_index:06d}.bin"
            merged.tofile(str(filename))
            print(f"Wrote {filename} with {merged.shape[0]} points from {GROUP} frames")
            kitti_index += 1
            buffer.clear()

    print("Conversion complete. KITTI files saved to", out_dir)


if __name__ == '__main__':
    main() 