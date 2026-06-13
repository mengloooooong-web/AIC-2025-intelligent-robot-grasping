import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
    get_package_share_directory('inuros2'),
    'config',
    'params2.yaml'
    )

    print(config)

    node=Node(
        package = 'inuros2',
        name = 'sensor_node2',
        executable = 'inuros2_camera_node',
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
        ],
        parameters = [config]
    )
    ld.add_action(node)
    return ld
