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
            executable='ligne',
            name='ligne',
            on_exit=launch.actions.Shutdown(),
            output='screen',
            emulate_tty=True,
            parameters = [{
                'DEBUG' : "1",
                }],  # To preserve format, color of output messages
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('challenge_project'), 'launch'),
            '/projet.launch.py']),
            launch_arguments=[
                        ("x_pose", '-2'),
                        ("y_pose", '0.59'),
                    ]
        ),
    ])
