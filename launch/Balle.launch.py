import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pj_ROS',
            executable='balle',
            name='balle',
            on_exit=launch.actions.Shutdown(),
            output='screen',  # To display output messages in the terminal
            emulate_tty=True,  # To preserve format, color of output messages
            parameters = [{
                "output_topic" : "/cmd_vel",
                "DEBUG" : "1",
                }],
        ),
        launch_ros.actions.Node(
            package='pj_ROS',
            executable='cible',
            name='cible',
            on_exit=launch.actions.Shutdown(),
            output='screen',  # To display output messages in the terminal
            emulate_tty=True,  # To preserve format, color of output messages
            parameters = [{
                "DEBUG" : "0",
                }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('challenge_project'), 'launch'),
            '/projet.launch.py']),
            launch_arguments=[
                        ("x_pose", '-5.3'), #-5.3
                        ("y_pose", '0.15'), #0.15
                    ]
        ),
    ])
