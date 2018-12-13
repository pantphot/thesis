import argparse
import sys
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.time_source import TimeSource
from rclpy.clock import Clock
from rclpy.clock import ClockType


from std_msgs.msg import Header

from sensor_msgs.msg import Image



class Mediator(Node):

    def __init__(self,qos_profile):
        super().__init__('mediator')

        self.timestamp=None
        self.throughput=[]
        self.myClock=Clock()
        self.myTime=Time()
        # signal.signal(signal.SIGINT, self.keyboardInterruptHandler)
        self.latencysec=[]
        self.count=0
        #init publisher
        #if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        #    self.pub = self.create_publisher(Header, 'image_data', qos_profile=qos_profile)
        #else:
        #    self.pub = self.create_publisher(Header, 'image_data_best_effort', qos_profile=qos_profile)


        if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('Reliable')
        else:
            self.get_logger().info('Best effort')
        print( self.image_callback, qos_profile)
        self.sub = self.create_subscription(Image, 'image', self.image_callback, qos_profile=qos_profile)


    def image_callback(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.header.frame_id)

        #Get received time
        self.timestamp = self.myClock.now()
        self.timestamp = self.timestamp.nanoseconds()
        print(self.timestamp)
        seconds = msg.header.stamp.sec
        nanoseconds = msg.header.stamp.nanosec

        #Calculate latency
        self.latencysec.append(float(self.timestamp[0]-seconds)+1.e-9*float(self.timestamp[1]-nanoseconds))
        self.count+=1
        #self.pub.publish(msg.header)

#     def keyboardInterruptHandler(self,signal, frame):
#         print (self.latencysec)
#         print ('Number of msgs received = ',self.count)
#         print ('Mean Latency = ',sum(self.latencysec)/self.count)
#         del node
#
# def __del__(self):
#     print ('destructor')

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--reliable', dest='reliable', action='store_true',
        help='set qos profile to reliable')
    parser.set_defaults(reliable=False)
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args(argv)
    rclpy.init(args=args.argv)

    if args.reliable:
        custom_qos_profile = qos_profile_default #reliable, keep last 10, volatile
    else:
        custom_qos_profile = qos_profile_sensor_data #best effort, keep last 5, volatile

    node = Mediator(custom_qos_profile)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print (node.latencysec)
        print ('Number of msgs received = ',node.count)
        print ('Mean Latency = ',sum(node.latencysec)/node.count)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
