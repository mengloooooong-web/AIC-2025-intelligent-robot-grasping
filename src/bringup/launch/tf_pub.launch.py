from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # tf camera_link->world
    cam_tf_node = Node(
        package="tf2_ros",
        name="static_transform_publisher",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "-1.570796", "0", "0", "Link_6", "camera_link"],
        # arguments=["0", "0", "2", "0", "3.141592", "0", "world", "camera_link"],
        # arguments = ['0', '0', '2', '1', '0', '0', '0', 'camera_link', 'world']
    )

    # box->camera_link
    box_tf_node = Node(
        package="tools_demo",
        name="tf_pub",
        executable="tf_pub",
    )

    ld.add_action(cam_tf_node)
    ld.add_action(box_tf_node)
    return ld
