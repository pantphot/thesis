import argparse
import sys
import signal
import time
import rclpy
import functools

from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.duration import Duration
from threading import Lock, Thread
import std_msgs
from std_msgs.msg import Header
import sensor_msgs.msg

logger = rclpy.logging.get_logger('monitor')
PERIOD = 1

class Monitor:
    def __init__(self,options,custom_qos_profile):
        self.time_arrived =[]
        self.options=options
        self.throughput=[]
        self.myClock=Clock()
        self.lock = Lock()
        self.latency=[]
        self.count=0
        self.time_sent=Time()
        self.qos_profile=custom_qos_profile

        #init publisher
        #if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        #    self.pub = self.create_publisher(Header, 'image_data', qos_profile=qos_profile)
        #else:
        #    self.pub = self.create_publisher(Header, 'image_data_best_effort', qos_profile=qos_profile)


    def topic_listening(self,node,options,qos_profile):
        print("begin topic listening")
        node_logger = node.get_logger()
        print("got logger")
        node_logger.info('Subscribing to topic: %s' % options.topic_name)
        sub = node.create_subscription(options.msg_type, options.topic_name,functools.partial(self.msg_callback, logger_=node_logger),qos_profile= qos_profile)

    def msg_callback(self, msg, logger_=logger):
        with self.lock:

            logger_.info('I heard: [%s]' % msg.header.frame_id)
        # with self.lock:
            self.time_sent=Time(seconds=msg.header.stamp.sec,nanoseconds=msg.header.stamp.nanosec)
            self.time_arrived=(self.myClock.now())
            self.latency.append((self.time_arrived.nanoseconds-self.time_sent.nanoseconds)*1e-9)
            self.count+=1
            # print(self.time_arrived)
            # print(self.time_var)
            print(self.latency)



    # def calculate_latency:
    #     with self.lock:
    #         self.latency.append(float(self.timestamp - seconds)+1.e-9*float(self.timestamp[1]-nanoseconds))

    #     #Calculate latency
    #     self.latencysec.append(float(self.timestamp[0]-seconds)+1.e-9*float(self.timestamp[1]-nanoseconds))
    #     self.count+=1
    #     #self.pub.publish(msg.header)
class DataReceivingThread(Thread):

    def __init__(self,monitor, options,custom_qos_profile):
        super(DataReceivingThread, self).__init__()
        rclpy.init()
        self.qos_profile=custom_qos_profile
        self.options = options
        self.monitor = monitor

    def run(self):
        self.node = rclpy.create_node('monitor')
        try:
            self.monitor.topic_listening(self.node, self.options,self.qos_profile)
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            print("Mean Latency = ",sum(monitor.latency)/monitor.count)
            self.stop()
            raise

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()



def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--reliable', dest='reliable', action='store_true',
        help='Set qos profile to reliable')
    parser.set_defaults(reliable=False)
    parser.add_argument(
        '-n', dest='topic_name', action='store',
        help='Name of topic to monitor')
    parser.add_argument(
        '-m', dest='msg_type', action='store',
        help='ROS msg type. **Capital first letter**')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args(argv)

    args.msg_type = getattr(std_msgs.msg,args.msg_type)

    if args.reliable:
        print('Reliable QoS policy')
        custom_qos_profile = qos_profile_default #reliable, keep last 10, volatile
    else:
        print('Best effort QoS policy')
        custom_qos_profile = qos_profile_sensor_data #best effort, keep last 5, volatile


    # if args.reliable:
    #
    # else:
    #

    monitor = Monitor(args,custom_qos_profile)
    print("monitor initialized")
    try:
        data_receiving_thread = DataReceivingThread(monitor,args,custom_qos_profile)
        data_receiving_thread.start()

        last_time = time.time()
        while data_receiving_thread.isAlive():
            now = time.time()
            if now - last_time > PERIOD:
                last_time = now
                #print(last_time)
                # monitor.calculate_latency()
                # topic_monitor.calculate_statistics()
                # if args.show_display:
                #     topic_monitor_display.update_display()
    finally:
        if data_receiving_thread.isAlive():
            data_receiving_thread.stop()
        # Block this thread until the other thread terminates
        data_receiving_thread.join()



if __name__ == '__main__':
    main()
