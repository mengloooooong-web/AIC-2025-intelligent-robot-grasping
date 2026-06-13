import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# /camera/aligned_depth_to_color/image_raw
# /camera/color/camera_info
# /camera/color/image_raw
# /depth/depth2pc
# /features/camera/fisheye/features
# /imu/sensor_msgs/Imu
# /objectdetection/vision_msgs/Detections
# /parameter_events
# /pointcloud/sensor/PointCloud2
# /rosout
# /sensor_msgs/Image/Fisheye
# /sensor_msgs/Image/Video/left/camera_info
# /sensor_msgs/Image/Video/left/image
# /sensor_msgs/Image/Video/right/camera_info
# /sensor_msgs/Image/Video/right/image
# /sensor_msgs/Image/camera_info
# /slam/sensor_msgs/Path


def generate_launch_description():
    ld = LaunchDescription()
    config1 = os.path.join(
        get_package_share_directory('inuros2'),
        'config',
        'params1.yaml'
    )

    node1=Node(
        package = 'inuros2',
        name = 'sensor_node1',
        executable = 'inuros2_camera_node',
        parameters = [config1],

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
        ]
    )

    config2 = os.path.join(
        get_package_share_directory('inuros2'),
        'config',
        'params2.yaml'
    )

    node2=Node(
        package = 'inuros2',
        name = 'sensor_node2',
        executable = 'inuros2_camera_node',
        parameters = [config2],
        remappings = [
            ('/camera/aligned_depth_to_color/image_raw', '/camera/aligned_depth_to_color/image_raw_2'),
            ('/camera/color/camera_info', '/camera/color/camera_info_2'),
            ('/camera/color/image_raw', '/camera/color/image_raw_2'),
            ('/depth/depth2pc', '/depth/depth2pc_2'),
            ('/features/camera/fisheye/features', '/features/camera/fisheye/features_2'),
            ('/imu/sensor_msgs/Imu', '/imu/sensor_msgs/Imu_2'),
            ('/objectdetection/vision_msgs/Detections', '/objectdetection/vision_msgs/Detections_2'),
            # ('/parameter_events', '/parameter_events_2'),
            ('/pointcloud/sensor/PointCloud2', '/pointcloud/sensor/PointCloud2_2'),
            # ('/rosout', '/rosout_2'),
            ('/sensor_msgs/Image/Fisheye', '/sensor_msgs/Image/Fisheye_2'),
            ('/sensor_msgs/Image/Video/left/camera_info', '/sensor_msgs/Image/Video/left/camera_info_2'),
            ('/sensor_msgs/Image/Video/left/image', '/sensor_msgs/Image/Video/left/image_2'),
            ('/sensor_msgs/Image/Video/right/camera_info', '/sensor_msgs/Image/Video/right/camera_info_2'),
            ('/sensor_msgs/Image/Video/right/image', '/sensor_msgs/Image/Video/right/image_2'),
            ('/sensor_msgs/Image/camera_info', '/sensor_msgs/Image/camera_info_2'),
            ('/slam/sensor_msgs/Path', '/slam/sensor_msgs/Path_2')
        ]
    )
    ld.add_action(node1)
    ld.add_action(node2)

    return ld
