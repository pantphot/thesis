#   latency.launch.py
#   Author: Pantelis Photiou
#   Created: Mar 2019
#
import argparse
import os
import sys
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.legacy import LaunchDescriptor
from launch.legacy.exit_handler import restart_exit_handler
from launch.legacy.launcher import DefaultLauncher
from launch.legacy.output_handler import ConsoleOutput
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor

    # package = 'nettools'
    # ld.add_process(
    #     cmd=[get_executable_path(package_name=package, executable_name='latency'),
    #     '-r','0','-m','image','-t','image'],
    #     name='nettools_latency',
    #     exit_handler=restart_exit_handler,
    # )
    package = 'nettools'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='nettools_plotter.py'),
         '-t', 'image','-s', 'latency'],
        name='latency_plotter',
        exit_handler=restart_exit_handler,
    )
    package = 'nettools'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='nettools_plotter.py'),
        '-s', 'msg_loss', '-t', 'image'],
        name='msg_loss_plotter',
        exit_handler=restart_exit_handler,
    )
    package = 'nettools'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='nettools_plotter.py'),
        '-s', 'frequency', '-t', 'image'],
        name='freq_plotter',
        exit_handler=restart_exit_handler,
    )
    package = 'nettools'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='nettools_plotter.py'),
        '-s', 'jitter', '-t', 'image'],
        name='jitter_plotter',
        exit_handler=restart_exit_handler,
    )

    return ld


def main(argv=sys.argv[1:]):
    launcher = DefaultLauncher()
    launch_descriptor = launch(LaunchDescriptor(), argv)
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
sys.exit(main())
