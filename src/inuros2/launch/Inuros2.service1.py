import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
    get_package_share_directory('inuros2'),
    'config',
    'params1.yaml'
    )

    print(config)

    node=Node(
        package = 'inuros2',
        name = 'sensor_node1',
        executable = 'inuros2_camera_node',
        # remap /camera/aligned_depth_to_color/image_raw to /camera_1/aligned_depth_to_color/image_raw
        remappings = [
            ('/camera/aligned_depth_to_color/image_raw', '/camera/aligned_depth_to_color/image_raw_1'),
            ('/camera/color/camera_info', '/camera/color/camera_info_1'),
            ('/camera/color/image_raw', '/camera/color/image_raw_1'),
            ('/depth/depth2pc', '/depth/depth2pc_1'),
            ('/features/camera/fisheye/features', '/features/camera/fisheye/features_1'),
            ('/imu/sensor_msgs/Imu', '/imu/sensor_msgs/Imu_1'),
            ('/objectdetection/vision_msgs/Detections', '/objectdetection/vision_msgs/Detections_1'),
            # ('/parameter_events', '/parameter_events_1'),
            ('/pointcloud/sensor/PointCloud2', '/pointcloud/sensor/PointCloud2_1'),
            # ('/rosout', '/rosout_1'),
            ('/sensor_msgs/Image/Fisheye', '/sensor_msgs/Image/Fisheye_1'),
            ('/sensor_msgs/Image/Video/left/camera_info', '/sensor_msgs/Image/Video/left/camera_info_1'),
            ('/sensor_msgs/Image/Video/left/image', '/sensor_msgs/Image/Video/left/image_1'),
            ('/sensor_msgs/Image/Video/right/camera_info', '/sensor_msgs/Image/Video/right/camera_info_1'),
            ('/sensor_msgs/Image/Video/right/image', '/sensor_msgs/Image/Video/right/image_1'),
            ('/sensor_msgs/Image/camera_info', '/sensor_msgs/Image/camera_info_1'),
            ('/slam/sensor_msgs/Path', '/slam/sensor_msgs/Path_1')
        ],
        parameters = [config]
    )
    ld.add_action(node)
    return ld
