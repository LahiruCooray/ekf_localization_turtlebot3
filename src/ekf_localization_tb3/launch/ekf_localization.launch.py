"""EKF Localization launch file for TurtleBot3 with GPS in Gazebo."""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ekf = get_package_share_directory('ekf_localization_tb3')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_ekf, 'worlds', 'empty_gps.sdf')
    model_path = os.path.join(pkg_ekf, 'models')
    model_file = os.path.join(model_path, 'turtlebot3_burger_gps', 'model.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    config_file = LaunchConfiguration('config', default='ekf_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_config = DeclareLaunchArgument(
        'config',
        default_value='ekf_params.yaml',
        description='Config file name (in config folder)'
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path + ':/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', model_file,
            '-name', 'burger',
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
        ],
        output='screen',
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[('/navsat', '/gps/fix')],
        output='screen',
    )

    ekf_node = Node(
        package='ekf_localization_tb3',
        executable='ekf_localization_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[
            [pkg_ekf, '/config/', config_file],
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_config,
        gz_resource_path,
        gazebo,
        delayed_spawn,
        bridge,
        ekf_node,
    ])

