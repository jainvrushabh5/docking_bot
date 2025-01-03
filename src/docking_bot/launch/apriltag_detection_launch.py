import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():

    apriltag_ros_share_dir = get_package_share_directory('apriltag_ros')

    tags_3611_yaml_file = os.path.join(apriltag_ros_share_dir,'cfg', 'tags_3611_yaml_file')


    launch_description = LaunchDescription([
        Node(
            package='apriltag_ros' ,
            executable = 'apriltag_node',
            name = 'apriltag_ros',
            output ='screen',
           remappings=[
               ('/image_rect', '/camera1/image_raw'),
               ('/camera_info', '/camera1/camera_info')
            ],
            parameters=[{'params_file': tags_3611_yaml_file}]
        ),
        Node(
            package='drobot_bot' ,
            executable = 'detection_subscriber.py',
            output = 'screen'
        )
    ])

    return launch_description