#   detection_localization.launch.py
#   Author: Pantelis Photiou
#   Created: Mar 2019
#

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='detection', node_executable='detection', output='screen',
                arguments=['-r', '0', '-s', '1']),
                #arguments=['-r 0', '-s 1']),
        launch_ros.actions.Node(
            package='detection', node_executable='position_estimation', output='screen'),
        #launch_ros.actions.Node(
        #    package='ros1_bridge', node_executable='dynamic_bridge', output='screen',
        #        arguments=['--bridge-all-topics']),
        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher', output='screen',
                arguments=['-1.44','0.91','1.1','0.0','0.0','0.0','1.0','map','external_camera']),
        ])
