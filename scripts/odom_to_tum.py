#!/usr/bin/env python3
"""Subscribe to nav_msgs/Odometry and save trajectory in TUM format (append mode)."""

import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry


class OdomToTum(Node):
    def __init__(self, topic, output, use_sim_time):
        super().__init__('odom_to_tum')
        if use_sim_time:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.output = output
        self.count = 0
        self.f = open(output, 'w')
        self.sub = self.create_subscription(Odometry, topic, self.cb, 10)
        self.get_logger().info(f'Subscribed to {topic}, writing to {output}')

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.f.write(f'{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n')
        self.count += 1
        if self.count % 100 == 0:
            self.f.flush()

    def close(self):
        if not self.f.closed:
            self.f.flush()
            self.f.close()
            self.get_logger().info(f'Saved {self.count} poses to {self.output}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/rko_lio/odometry')
    ap.add_argument('--output', required=True)
    ap.add_argument('--use-sim-time', default='true')
    args = ap.parse_args()

    # Let rclpy install its own SIGINT/SIGTERM handlers — custom signal.signal()
    # handlers do not interrupt rclpy.spin() because rcl_wait blocks in C and
    # never yields to Python signal processing.
    rclpy.init(args=sys.argv)
    node = OdomToTum(args.topic, args.output,
                     args.use_sim_time.lower() in ('true', '1', 'yes'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
