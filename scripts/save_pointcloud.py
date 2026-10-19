#!/usr/bin/env python3
# /*******************************************************************************
# * Copyright 2025 Intelligents Robotics Lab - URJC.
# *
# *******************************************************************************/

# /* Author: Juan S. Cely */

"""Save one message of a PointCloud2 topic as a binary PCD file.

Usage (with the simulation and the bridge running):

    python3 scripts/save_pointcloud.py --ros-args \
        -p topic:=/ray_tracing_cloud -p output:=maps/ray_tracing.pcd
"""

from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudSaver(Node):

    def __init__(self):
        super().__init__('save_pointcloud')
        self.topic = self.declare_parameter('topic', '/ray_tracing_cloud').value
        self.output = Path(self.declare_parameter('output', 'map.pcd').value).expanduser()
        self.done = False
        self.create_subscription(PointCloud2, self.topic, self.callback, 1)
        self.get_logger().info(f'Waiting for a point cloud on {self.topic}')

    def callback(self, msg):
        if self.done:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        write_pcd(self.output, points.astype(np.float32))
        self.get_logger().info(
            f'{len(points)} points (frame {msg.header.frame_id}) saved to {self.output}')
        self.done = True


def write_pcd(path, points):
    """Write an N x 3 float32 array as a binary PCD v0.7 file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(np.ascontiguousarray(points, dtype='<f4').tobytes())


def main():
    rclpy.init()
    node = PointCloudSaver()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
