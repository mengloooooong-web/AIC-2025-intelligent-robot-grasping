import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
                            get_package_share_directory('inuros2'),
                            'config',
                            'params.yaml'
                        )

    with open(config,'r') as f:
        params = yaml.safe_load(f)['sensor_node']['ros__parameters']

    my_component = ComposableNode(
        package='inuros2',
        plugin='inuros2::CInuDevRosNodeFactory',
        name='sensor_node',
    )

    container = ComposableNodeContainer(
        name='inu_container',
        parameters = [params],
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[my_component],
        output='screen')

    return LaunchDescription([container])
