#!/usr/bin/env python3

# license removed for brevity
# This scripts runs a node that subscribes on topic statistics and plots the given network statistic
# Created by Pantelis Photiou, at Jan 2018
import argparse
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from std_msgs.msg import Float64
from nettools_msgs.msg import TopicStatistics
from nettools_msgs.msg import StatisticsMeasurements

class NettoolsPlotter(Node):
    def __init__(self,stat):
        super().__init__('nettools_plotter')
        self.stat = stat
        self.x_data = []
        self.y_data = []
        self.y_data_val = []
        self.y_data_avg = []
        self.y_data_std = []
        self.y_data_min = []
        self.y_data_max = []
        self.count = 0
        self.fig = plt.figure()
        plt.title(self.stat)
        if self.stat == 'throughput':
            self.sub = self.create_subscription(Float64, 'topic_statistics', self.plot_callback,qos_profile=qos_profile_default)
            plt.xlabel('Time (s)')
            plt.ylabel('Throughput (Mbps)')
            plt.ylim(0.0,10.0)
        elif self.stat == 'latency':
            self.sub = self.create_subscription(TopicStatistics, 'topic_statistics', self.plot_callback,qos_profile=qos_profile_default)
            plt.xlabel('Messages Received')
            plt.ylabel('Latency (ms)')
            plt.ylim(0.0,500.0)
        elif self.stat == 'frequency':
            self.sub = self.create_subscription(TopicStatistics, 'topic_statistics', self.plot_callback,qos_profile=qos_profile_default)
            plt.xlabel('Messages Received')
            plt.ylabel('Frequency (Hz)')
            plt.ylim(0.0,5.0)
        else:
            self.sub = self.create_subscription(TopicStatistics, 'topic_statistics', self.plot_callback,qos_profile=qos_profile_default)
            plt.xlabel('Messages Received')
            plt.ylabel(self.stat)
            plt.ylim(0.0,200.0)
        # plt.xlim(0,10000)
        print('sub created')
        plt.grid(True)
        self.ax = self.fig.add_subplot(111)
        if (self.stat == 'latency' or self.stat == 'frequency'):
            self.val, = self.ax.plot([],[],color= 'r',label=str(self.stat))
            self.avg, = self.ax.plot([],[],color= 'b',label=str(self.stat + ' avg'))
            self.std, = self.ax.plot([],[],color= 'g',label=str(self.stat + ' std'))
            self.min, = self.ax.plot([],[],color= 'y',label=str(self.stat + ' min'))
            self.max, = self.ax.plot([],[],color= 'c',label=str(self.stat + ' max'))
        else:
            self.line, = self.ax.plot([],[],color= 'r',label=self.stat)

        self.ax.legend(
            loc='upper center', bbox_to_anchor=(0.5, -0.1), fancybox=True, shadow=True, ncol=2)
        plt.show(block=False)
        # Shrink axis' height to make room for legend
        shrink_amnt = 0.2
        box = self.ax.get_position()
        self.ax.set_position(
            [box.x0, box.y0 + box.height * shrink_amnt, box.width, box.height * (1 - shrink_amnt)])
        print('init finished')

    def plot_callback(self, msg):
        self.count +=1
        self.x_data.append(self.count)
        if (self.stat == 'latency'):
            self.y_data_val.append(msg.latency.val)
            self.val.set_ydata(self.y_data_val)
            self.y_data_avg.append(msg.latency.avg)
            self.avg.set_ydata(self.y_data_avg)
            self.y_data_std.append(msg.latency.std)
            self.std.set_ydata(self.y_data_std)
            self.y_data_min.append(msg.latency.min)
            self.min.set_ydata(self.y_data_min)
            self.y_data_max.append(msg.latency.max)
            self.max.set_ydata(self.y_data_max)

            self.val.set_xdata(self.x_data)
            self.avg.set_xdata(self.x_data)
            self.std.set_xdata(self.x_data)
            self.min.set_xdata(self.x_data)
            self.max.set_xdata(self.x_data)
        elif (self.stat == 'frequency'):
            self.y_data_val.append(msg.frequency.val)
            self.val.set_ydata(self.y_data_val)
            self.y_data_avg.append(msg.frequency.avg)
            self.avg.set_ydata(self.y_data_avg)
            self.y_data_std.append(msg.frequency.std)
            self.std.set_ydata(self.y_data_std)
            self.y_data_min.append(msg.frequency.min)
            self.min.set_ydata(self.y_data_min)
            self.y_data_max.append(msg.frequency.max)
            self.max.set_ydata(self.y_data_max)
            self.val.set_xdata(self.x_data)
            self.avg.set_xdata(self.x_data)
            self.std.set_xdata(self.x_data)
            self.min.set_xdata(self.x_data)
            self.max.set_xdata(self.x_data)
        elif (self.stat == 'msg_loss'):
            self.y_data.append(msg.msg_loss)
            # logger_.info('%d' % (msg.msg_loss))
            self.line.set_ydata(self.y_data)
            self.line.set_xdata(self.x_data)
        elif (self.stat == 'jitter'):
            self.y_data.append(msg.jitter)
            # logger_.info('%f' % (msg.jitter))
            self.line.set_ydata(self.y_data)
            self.line.set_xdata(self.x_data)
        elif (self.stat == 'throughput'):
            self.y_data.append(msg.data)
            # logger_.info('%f' % (msg.data))
            self.line.set_ydata(self.y_data)
            self.line.set_xdata(self.x_data)

        self.ax.relim()
        self.ax.autoscale_view(True,True,True)
        self.fig.canvas.draw()
        plt.pause(0.0001)

# ######################################################################
def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'stat', choices=['msg_loss' , 'jitter' , 'latency' , 'frequency', 'throughput'], action='store',
        help='set the network statistic to be plotted')
    parser.set_defaults(stat='latency')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args(argv)

    rclpy.init(args=args.argv)

    print (args.stat)

    node = NettoolsPlotter(args.stat)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print ("Shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
