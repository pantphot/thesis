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
        '-r', '0', '-s', '1'],
        name='detection_node',
        exit_handler=restart_exit_handler,
    )
    package = 'detection'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='position_estimation')],
        name='goal_publisher_node',
        exit_handler=restart_exit_handler,
    )
    package = 'tf2_ros'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='static_transform_publisher'),
        '-1.58','0.95','1.11','0.0','0.0','0.0','1.0','map','external_camera'],
        name='static_tf_pub_map_external',
        exit_handler=restart_exit_handler,
    )
    package = 'ros1_bridge'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='dynamic_bridge'), '--bridge-all-topics'],
        name='ros_bridge',
        exit_handler=restart_exit_handler,
        output_handlers=[ConsoleOutput()],
    )

    return ld
        # launch_ros.actions.Node(
        #     package='detection', node_executable='detection', output='screen',
        #         arguments=['-r', '0', '-s', '1']),
        #         #arguments=['-r 0', '-s 1']),
        # launch_ros.actions.Node(
        #     package='detection', node_executable='position_estimation', output='screen'),
        # #launch_ros.actions.Node(
        # #    package='ros1_bridge', node_executable='dynamic_bridge', output='screen',
        # # #        arguments=['--bridge-all-topics']),
        # launch_ros.actions.Node(
        #     package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        #         #arguments=['-1.44','0.91','1.1','0.0','0.0','0.0','1.0','map','external_camera']),
        #         arguments=['-1.602','1.047','1.11','0.0','0.0','0.0','1.0','map','external_camera']),
        # ])

def main(argv=sys.argv[1:]):
    launcher = DefaultLauncher()
    launch_descriptor = launch(LaunchDescriptor(), argv)
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
