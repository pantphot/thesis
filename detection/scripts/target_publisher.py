#!/usr/bin/env python
#
#
#
import argparse
import sys
import rclpy
from rclpy.node import Node

import tf2_ros
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import PoseStamped

class TargetPublisher(Node):
    def __init__(self,custom_qos_profile):
        super().__init__('target_publisher')
        self.custom_qos_profile = custom_qos_profile
        self.pub = self.create_publisher(PoseStamped, 'move_base_simple/goal', qos_profile=self.custom_qos_profile)

        self.sub = create_subscription(PoseStamped, 'detected_point'  , self.callback,qos_profile=self.custom_qos_profile)

    def callback(self,msg):


        # Let src_pt be the point you want to transform.
        tf_buf = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buf)
        target_pt = tf_buf.transform(msg, "map")

def main(argv=sys.argv[1:]):
    # parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # parser.add_argument(
    #     '-s',dest='stat', choices=['msg_loss' , 'jitter' , 'latency' , 'frequency', 'throughput'], action='store',
    #     help='set the network statistic to be plotted')
    # parser.add_argument(
    #     '-t',dest='topic',action='store',
    #     help='set the topic where the statistics are calculated from')
    # parser.set_defaults(stat='latency')
    # parser.add_argument(
    #     'argv', nargs=argparse.REMAINDER,
    #     help='Pass arbitrary arguments to the executable')
    # args = parser.parse_args(argv)
    # parser.print_help()
    rclpy.init(args=args.argv)

    # print (args.topic)
    # print (args.stat)
    custom_qos_profile = qos_profile_default
    node = TargetPublisher(custom_qos_profile)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print ("Shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
