import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ros2 launch jaka_planner moveit_server.launch.py ip:=10.5.5.100 model:=minicobo
    # ros2 launch jaka_minicobo_moveit_config demo.launch.py

    # moveit_server
    jaka_ip = DeclareLaunchArgument(
        "ip", default_value=TextSubstitution(text="10.5.5.100")
    )
    jaka_model = DeclareLaunchArgument(
        "model", default_value=TextSubstitution(text="minicobo")
    )

    moveit_server_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jaka_planner"),
                "launch/moveit_server.launch.py",
            )
        ),
        launch_arguments={
            "ip": LaunchConfiguration("ip"),
            "model": LaunchConfiguration("model"),
        }.items(),
    )

    # moveit_config
    moveit_config_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jaka_minicobo_moveit_config"),
                "launch/demo.launch.py",
            )
        )
    )
    # ros2 run jaka_planner moveit_test_2 --ros-args -p model:=minicobo -p x_position:=-0.35 -p y_position:=0.01 -p z_position:=0.130 -p rx:=3.14159 -p ry:=0.0 -p rz:=0.0
    # 初始化位置
    # moveit_test = Node(
    #     package="jaka_planner",
    #     executable="moveit_test_2",
    #     name="moveit_test_2",
    #     parameters=[
    #         {"model": LaunchConfiguration("model")},
    #         {"x_position": -0.35},
    #         {"y_position": 0.01},
    #         {"z_position": 0.130},
    #         {"rx": 3.14159},
    #         {"ry": 0.0},
    #         {"rz": 0.0},
    #     ],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            jaka_ip,
            jaka_model,
            moveit_server_node,
            moveit_config_node,
            # moveit_test,
            # moveit_test_2_node,
        ]
    )
