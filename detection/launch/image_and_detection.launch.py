#   detection_localization.launch.py
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
    package = 'detection'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='detection'),
        '-r', '0', '-s', '0'],
        name='detector',
        exit_handler=restart_exit_handler,
    )
    package = 'image_tools'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='cam2image'),
        '-x', '352', '-y', '288','-r', '0','-f', '5'],
        name='cam2image',
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
