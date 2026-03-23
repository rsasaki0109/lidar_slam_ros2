#!/usr/bin/env python3
"""Subscribe to nav_msgs/Path and save the latest path as TUM format."""

import argparse
import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Path


class PathToTum(Node):
    def __init__(self, topic, output, use_sim_time):
        super().__init__('path_to_tum')
        if use_sim_time:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.output = output
        self.sub = self.create_subscription(Path, topic, self.cb, 10)
        self.get_logger().info(f'Subscribed to {topic}, will save to {output}')

    def cb(self, msg):
        self.get_logger().info(f'Received path with {len(msg.poses)} poses')
        # Write immediately on each path received (overwrites previous)
        with open(self.output, 'w') as f:
            for ps in msg.poses:
                t = ps.header.stamp.sec + ps.header.stamp.nanosec * 1e-9
                p = ps.pose.position
                q = ps.pose.orientation
                f.write(f'{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/modified_path')
    ap.add_argument('--output', required=True)
    ap.add_argument('--use-sim-time', default='true')
    args = ap.parse_args()

    rclpy.init()
    node = PathToTum(args.topic, args.output,
                     args.use_sim_time.lower() in ('true', '1', 'yes'))

    def signal_handler(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
