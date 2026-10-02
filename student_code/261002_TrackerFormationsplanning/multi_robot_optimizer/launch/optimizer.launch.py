import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Pfad zur Parameter-Datei finden
    config = os.path.join(
        get_package_share_directory('multi_robot_optimizer'),
        'config',
        'pso_params.yaml'
    )

    optimizer_node = Node(
        package='multi_robot_optimizer',
        executable='optimizer_node',
        name='formation_optimizer_node',
        output='screen',
        parameters=[config] # Hier werden deine LaTeX-Variablen geladen!
    )

    return LaunchDescription([
        optimizer_node
    ])