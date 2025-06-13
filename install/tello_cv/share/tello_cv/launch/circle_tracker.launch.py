from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    target_radius_arg = DeclareLaunchArgument('target_radius', default_value='60')
    target_radius = LaunchConfiguration('target_radius')
    scale_arg = DeclareLaunchArgument('scale', default_value='0.75')
    scale = LaunchConfiguration('scale')
    max_tracker_err_arg = DeclareLaunchArgument('max_tracker_err', default_value='20')
    max_tracker_err = LaunchConfiguration('max_tracker_err')

    circle_tracker_node = Node(
        package='tello_cv',
        executable='circle_tracker_node',
        name='circle_tracker_node',
        output='screen',
        parameters=[{'scale_param': scale, 'max_tracker_err_param': max_tracker_err, 'target_radius_param': target_radius}]
    )

    pid_controller_node = Node(
        package='tello_cv',
        executable='pid_controller_node',
        name='pid_controller_node',
        output='screen',
        parameters=[{'target_radius_param': target_radius}]
    )

    ld = LaunchDescription([
        target_radius_arg,
        scale_arg,
        max_tracker_err_arg,
        circle_tracker_node,
        pid_controller_node,
    ])

    return ld