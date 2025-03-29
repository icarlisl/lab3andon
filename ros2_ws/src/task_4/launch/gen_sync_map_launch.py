from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('turtlebot4_navigation'),
                '/launch/slam.launch.py'
            ]),
            launch_arguments={
                'namespace': '/robot',
                'odom_frame': 'odom',
                'map_frame': 'map'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('turtlebot4_viz'),
                '/launch/view_robot.launch.py'
            ]),
            launch_arguments={'namespace': '/robot'}.items()
        ),
    ])

