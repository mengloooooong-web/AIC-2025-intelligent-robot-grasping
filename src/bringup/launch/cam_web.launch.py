import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # ros2 launch inuros2 Inuros2.launch.py
    cam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("inuros2"), "launch/Inuros2.launch.py"
            )
        )
    )

    websocket_smart_topic_arg = DeclareLaunchArgument(
        "websocket_smart_topic",
        default_value=TextSubstitution(text="not_use"),
    )
    websocket_only_show_image_arg = DeclareLaunchArgument(
        "websocket_only_show_image",
        default_value=TextSubstitution(text="True"),
    )
    websocket_image_topic_arg = DeclareLaunchArgument(
        "websocket_image_topic",
        default_value=TextSubstitution(text="/camera/color/image_raw/compressed"),
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"), "launch/websocket.launch.py"
            )
        ),
        launch_arguments={
            "websocket_image_topic": LaunchConfiguration("websocket_image_topic"),
            "websocket_image_type": "mjpeg",
            "websocket_only_show_image": LaunchConfiguration(
                "websocket_only_show_image"
            ),
            "websocket_smart_topic": LaunchConfiguration("websocket_smart_topic"),
        }.items(),
    )

    # jpeg->nv12
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_codec"),
                "launch/hobot_codec_decode.launch.py",
            )
        ),
        launch_arguments={
            "codec_in_mode": "ros",
            "codec_sub_topic": LaunchConfiguration("websocket_image_topic"),
            "codec_out_mode": "shared_mem",
            "codec_pub_topic": "/hbmem_img",
        }.items(),
    )

    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        )
    )

    return LaunchDescription(
        [
            # 图片发布pkg
            cam_node,
            # web展示pkg
            websocket_smart_topic_arg,
            websocket_only_show_image_arg,
            websocket_image_topic_arg,
            web_node,
            # 图像编解码
            nv12_codec_node,
            # 启动零拷贝环境配置节点
            shared_mem_node,
        ]
    )
