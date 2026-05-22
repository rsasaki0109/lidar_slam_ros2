#!/usr/bin/env python3
"""Subscribe to nav_msgs/Path and save the latest path as TUM format."""

import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
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

    # Let rclpy install its own SIGINT/SIGTERM handlers (signal_handler_options
    # defaults to ALL). Custom signal.signal() handlers do not interrupt
    # rclpy.spin() because rcl_wait blocks in C and never yields to Python
    # signal processing — we discovered this when the dogfood wrapper hung
    # forever on `kill -INT`.
    rclpy.init(args=sys.argv)
    node = PathToTum(args.topic, args.output,
                     args.use_sim_time.lower() in ('true', '1', 'yes'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
