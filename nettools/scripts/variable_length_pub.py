#!/usr/bin/env python3
    # This scripts runs a node that publishes variable length messages
# Created by Pantelis Photiou, at Jan 2018

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.clock import Clock
from std_msgs.msg import Header
from collections.abc import Set
import timeit

from nettools_msgs.msg import ByteArray



class VariableLengthPub(Node):

    def __init__(self, custom_qos_profile,freq,step):
        super().__init__('variable_length_pub')
        self.i=0
        self.step = step
        self.clock = Clock()
        self.msg = ByteArray()
        self.data = [0]*1024

        if custom_qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('Reliable publisher')
        else:
            self.get_logger().info('Best effort publisher')
        self.pub = self.create_publisher(ByteArray, 'variable_length', qos_profile=custom_qos_profile)

        timer_period = 1 / freq
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.i += 1
        self.msg.header.frame_id = str(self.i)
        x = 0
        #offset 24 bytes (size of header)
        for x in range(1024*self.step):
            self.msg.data.append(bytes(1))

        self.get_logger().info('Publishing message: "{0}"'.format(self.msg.header.frame_id))
        timestamp = self.clock.now()
        self.msg.header.stamp = timestamp.to_msg()
        self.pub.publish(self.msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--reliable', dest='reliable', action='store_true',
        help='set qos profile to reliable')
    parser.set_defaults(reliable=False)
    parser.add_argument(
        '-f', type=int, default=30, dest='freq',action='store',
        help='set publishing frequency in Hz. 30(default) ')
    parser.add_argument(
        '-s', type=int, default=1, dest='step',action='store',
        help='set message length increment step in kB. (default = 1kB) ')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args(argv)
    rclpy.init(args=args.argv)

    if args.reliable:
        custom_qos_profile = qos_profile_default
        print("Reliable")
    else:
        custom_qos_profile = qos_profile_sensor_data
        print("Best effort")
    node = VariableLengthPub(custom_qos_profile,args.freq,args.step)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print ("Shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
