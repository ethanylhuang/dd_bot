import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_path = get_package_share_directory('dd_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    urdf = xacro.process_file(xacro_file).toxml()

    urdf_file = '/tmp/dd_bot.urdf'
    with open(urdf_file, 'w') as f:
        f.write(urdf)


    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'dd_bot',
            '-file', urdf_file
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_launch,
        rsp,
        spawn
    ])
