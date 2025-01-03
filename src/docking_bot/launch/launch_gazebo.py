import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for initial pose
    initial_x = DeclareLaunchArgument('x', default_value='0.0', description='Initial X position of the robot')
    initial_y = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y position of the robot')
    initial_z = DeclareLaunchArgument('z', default_value='0.0', description='Initial Z position of the robot')
    initial_roll = DeclareLaunchArgument('roll', default_value='0.0', description='Initial roll of the robot')
    initial_pitch = DeclareLaunchArgument('pitch', default_value='0.0', description='Initial pitch of the robot')
    initial_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw of the robot')

 
    package_name = 'docking_bot' 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'robot.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

 
    custom_world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'cube_world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': custom_world_path}.items()
    )

    # Spawn entity with initial pose
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen'
    )

    # Launch everything
    return LaunchDescription([
        initial_x,
        initial_y,
        initial_z,
        initial_roll,
        initial_pitch,
        initial_yaw,
        rsp,
        gazebo,
        spawn_entity,
    ])
