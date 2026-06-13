from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """ps
    需要启动
    llama-server -m /home/sunrise/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf -c 2048 --threads 8 --port 8081 &
    """

    pose_get = Node(
        package="tools_demo",
        executable="pose_get",
        output="screen",
        # arguments=["--ros-args", "--log-level", "warn"],
    )
    connecter = Node(
        package="tools_demo",
        executable="connecter",
        output="screen",
        # arguments=["--ros-args", "--log-level", "warn"],
    )
    return LaunchDescription([pose_get, connecter])
