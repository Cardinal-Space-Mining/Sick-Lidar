import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('sick_perception')
    param_file = os.path.join(pkg_path, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_node = Node(
        name = 'sick_perception',
        package = 'sick_perception',
        executable = 'sick_perception',
        output = 'screen',
        parameters = [param_file, {'use_sim_time': use_sim_time}],
        remappings = [
            ('obstacle_grid', '/sick_perception/obstacle_grid')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        map_node
    ])
