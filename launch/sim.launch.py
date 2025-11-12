import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_name = "dd_bot"
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_path = get_package_share_directory('dd_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    urdf = xacro.process_file(xacro_file).toxml()

    urdf_file = '/tmp/dd_bot.urdf'
    with open(urdf_file, 'w') as f:
        f.write(urdf)

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty_custom.world'
    )    
    
    world = LaunchConfiguration('world')
        
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

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
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -v4 '), world]
        }.items()
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'dd_bot',
            '-file', urdf_file
        ],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )



    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        world_arg,
        gz_launch,
        rsp,
        clock_bridge,
        spawn,
        diff_drive_spawner,
        joint_broad_spawner
    ])
