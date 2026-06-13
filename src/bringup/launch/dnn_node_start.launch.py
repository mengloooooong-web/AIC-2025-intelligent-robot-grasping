# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # 拷贝config中文件
    dnn_node_example_path = os.path.join(get_package_share_directory("bringup"))
    print("dnn_node_example_path is ", dnn_node_example_path)
    cp_cmd = "cp -rs " + dnn_node_example_path + "/config ."
    print("cp_cmd is ", cp_cmd)
    os.system(cp_cmd)

    # args that can be set from the command line or a default will be used
    config_file_launch_arg = DeclareLaunchArgument(
        "dnn_example_config_file",
        default_value=TextSubstitution(text="config/yolov5nblockconfig.json"),
    )
    dump_render_launch_arg = DeclareLaunchArgument(
        "dnn_example_dump_render_img", default_value=TextSubstitution(text="0")
    )
    image_width_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_width", default_value=TextSubstitution(text="800")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_height", default_value=TextSubstitution(text="608")
    )
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "dnn_example_msg_pub_topic_name",
        default_value=TextSubstitution(text="hobot_dnn_detection"),
    )
    websocket_only_show_image_arg = DeclareLaunchArgument(
        "websocket_only_show_image",
        default_value=TextSubstitution(text="False"),
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bringup"), "launch/cam_web.launch.py"
            )
        ),
        launch_arguments={
            "websocket_only_show_image": LaunchConfiguration(
                "websocket_only_show_image"
            ),
            "websocket_smart_topic": LaunchConfiguration(
                "dnn_example_msg_pub_topic_name"
            ),
        }.items(),
    )

    # 算法pkg
    dnn_node_example_node = Node(
        package="dnn_node_example",
        executable="example",
        output="screen",
        parameters=[
            {"config_file": LaunchConfiguration("dnn_example_config_file")},
            {"dump_render_img": LaunchConfiguration("dnn_example_dump_render_img")},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {
                "msg_pub_topic_name": LaunchConfiguration(
                    "dnn_example_msg_pub_topic_name"
                )
            },
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )
    # tf camera_link->world
    cam_tf_node = Node(
        package="tf2_ros",
        name="static_transform_publisher",
        executable="static_transform_publisher",
        arguments=[
            "-0.0554",
            "0",
            "0.007",
            # "-1.570796",
            "0",
            "0",
            "0",
            "Link_6",
            "camera_link",
        ],
    )
    # arguments=["0", "0", "2", "0", "3.141592", "0", "world", "camera_link"],
    # arguments = ['0', '0', '2', '1', '0', '0', '0', 'camera_link', 'world']

    ld = LaunchDescription()
    # export CAM_TF=True
    cam_tf = os.getenv("CAM_TF")
    print("camera_tf is ", cam_tf)
    if cam_tf == "True":
        ld.add_action(cam_tf_node)
        print("add cam_tf_node")

    ld.add_action(config_file_launch_arg)
    ld.add_action(dump_render_launch_arg)
    ld.add_action(image_width_launch_arg)
    ld.add_action(image_height_launch_arg)
    ld.add_action(websocket_only_show_image_arg)
    ld.add_action(msg_pub_topic_name_launch_arg)
    ld.add_action(dnn_node_example_node)
    ld.add_action(web_node)
    return ld
