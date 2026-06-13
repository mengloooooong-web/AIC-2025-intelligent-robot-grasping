from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """LangGraph Agent 启动文件

    基于 LangGraph 框架的 Agent 决策系统启动文件

    需要启动的服务：
    1. llama-server -m /home/sunrise/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf -c 2048 --threads 8 --port 8081 &
    2. DNN 检测节点 (dnn_node_start.launch.py)
    3. 机械臂驱动 (jaka_start.launch.py)

    服务接口：
    - /agent_send_command: 发送抓取命令 (StrMsg)
      示例: ros2 service call /agent_send_command custom_msgs/srv/StrMsg "{data: '{\"color\": \"yellow\", \"num\": 3}'}"
    - /restart_detection: 重启检测服务 (std_srvs/Trigger)
    """

    pose_get = Node(
        package="tools_demo",
        executable="pose_get",
        output="screen",
    )

    langgraph_agent = Node(
        package="tools_demo",
        executable="langgraph_agent",
        output="screen",
        parameters=[
            {"agent_max_retries": 3},
            {"detection_threshold": 0.55},
        ],
    )

    connecter = Node(
        package="tools_demo",
        executable="connecter",
        output="screen",
    )

    return LaunchDescription([
        pose_get,
        langgraph_agent,
        connecter,
    ])