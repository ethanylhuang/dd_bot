from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')


    # or just run this "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix=['xterm ', '-e'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_teleop')],
        output='screen'
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_in',  '/diff_cont/cmd_vel_unstamped'),
            ('/cmd_vel_out', '/diff_cont/cmd_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        teleop_keyboard,
        twist_stamper
    ])
