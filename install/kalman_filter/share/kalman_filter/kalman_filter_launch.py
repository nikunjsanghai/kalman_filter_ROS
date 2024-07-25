from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kalman_filter',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', 'bag_files/highway_drive_bag'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'kalman_filter_results', '/imu/data', '/gps/data', '/baseline/data', '/kalman_filter/estimate'],
            output='screen'
        )
    ])
