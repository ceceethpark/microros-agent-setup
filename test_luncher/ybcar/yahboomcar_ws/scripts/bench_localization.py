#!/usr/bin/env python3
"""Simple ROS2 node to log odometry and AMCL poses to CSV for offline analysis.

Usage (after sourcing ROS2 + workspace):
  python3 scripts/bench_localization.py --duration 30 --out logs/loc_log.csv

This requires `rclpy` available in the environment where it's run.
"""
import argparse
import csv
import os
import time
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0, help='seconds to record')
    parser.add_argument('--out', type=str, default='logs/loc_log.csv')
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
    except Exception as e:
        print('This script must be run in a ROS2 python environment with rclpy installed:', e)
        return

    class Recorder(Node):
        def __init__(self, out_path):
            super().__init__('bench_localization_recorder')
            self.out_path = out_path
            self.file = open(out_path, 'w', newline='', encoding='utf-8')
            self.csvw = csv.writer(self.file)
            self.csvw.writerow(['ts', 'topic', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.cb_odom, 10)
            self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)

        def cb_odom(self, msg: Odometry):
            t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.csvw.writerow([t, '/odometry/filtered', p.x, p.y, p.z, q.x, q.y, q.z, q.w])

        def cb_amcl(self, msg: PoseWithCovarianceStamped):
            t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.csvw.writerow([t, '/amcl_pose', p.x, p.y, p.z, q.x, q.y, q.z, q.w])

        def close(self):
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass

    rclpy.init()
    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    node = Recorder(str(outp))
    start = time.time()
    try:
        while time.time() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
