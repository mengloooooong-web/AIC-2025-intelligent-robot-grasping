# from time import sleep
"""
订阅检测结果：
dnn 检测到的方块信息
彩色图像
深度图像

计算：
相机坐标系下的方块位置
转换
工具坐标系下的方块位置
方块颜色

服务返回：
工具坐标系下的方块位置
方块颜色
"""

import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets, Target, Roi
from sensor_msgs.msg import Image, CompressedImage

import numpy as np
from cv_bridge import CvBridge

from tools_demo.modules.tools import (
    get_color,
    pixel_to_camera,
    create_handeye_matrix,
    camera_to_tool,
)

from std_srvs.srv import Trigger
import json

import cv2


class DetectionTFBroadcaster(Node):
    def __init__(self):
        super().__init__("block_pose_get")
        # 订阅检测结果
        self.subscription = self.create_subscription(
            PerceptionTargets, "/hobot_dnn_detection", self.detection_callback, 10
        )

        # 深度图像
        self.depth_subscription = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.depth_image_callback,
            10,
        )

        # 彩色图像
        self.color_subscription = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.color_image_callback,
            10,
        )

        self.color_image_ = None
        # self.pub = self.create_publisher(String, "/block_pose", 10)

        # 图像参数
        self.depth_image = None
        self.bridge = CvBridge()

        self.T_tool_cam = create_handeye_matrix()

        # 检测状态控制
        self.detected_blocks = []  # 存储检测到的方块信息
        self.detected_blocks_str = ""  # 存储检测到的方块信息的字符串表示
        self.target_count = 0  # 检测的方块数量

        self.declare_parameter("target_count", self.target_count)
        self.declare_parameter("detected_blocks", "")
        # 识别服务
        self.restart_detection_service = self.create_service(
            Trigger, "/restart_detection", self.restart_detection_callback
        )

    def detection_callback(self, msg: PerceptionTargets):
        # if self.detection_complete:
        #     return  # 如果检测已完成，跳过处理
        self.detected_blocks = []  # 清空之前的检测结果
        self.target_count = 0  # 重置目标计数

        # 等待深度图像
        if self.depth_image is None:
            self.get_logger().info("Depth image not available yet")
            return

        new_blocks = []  # 本次回调检测到的新方块

        for target in msg.targets:
            target: Target
            for roi in target.rois:
                roi: Roi
                if roi.type != "block" or roi.confidence < 0.55:
                    continue

                # 计算中心像素坐标（彩色图像）
                x_offset = roi.rect.x_offset
                y_offset = roi.rect.y_offset
                width = roi.rect.width
                height = roi.rect.height
                center_x_color = x_offset + width / 2
                center_y_color = y_offset + height / 2

                # 将彩色坐标转换到深度图像坐标系
                u_depth, v_depth = center_x_color, center_y_color
                # resize_pixel_coordinates(
                #     center_x_color, center_y_color
                # )
                # 校正畸变
                u_undistorted, v_undistorted = u_depth, v_depth
                #  correct_distortion(u_depth, v_depth)
                # 获取深度值
                depth_value = self.get_depth_value(u_undistorted, v_undistorted)
                if depth_value is None:
                    # continue
                    depth_value = 0.0
                # 获取颜色
                color: str = self.get_color_([x_offset, y_offset, width, height])
                if color == "no_image":
                    #
                    color = "no_image"
                # 转换为3D坐标
                point_3d = pixel_to_camera(u_undistorted, v_undistorted, depth_value)

                # 将相机坐标系下的3D点转换为工具坐标系下的3D点
                point_3d_tool = camera_to_tool(
                    point_3d[0], point_3d[1], point_3d[2], self.T_tool_cam
                )

                # 检查是否已检测到类似位置的方块（避免重复）
                if not self.is_new_block(point_3d_tool):
                    continue

                # 创建方块信息
                block_info = {
                    "id": len(self.detected_blocks) + len(new_blocks),
                    "tool_pos": [point_3d_tool[0], point_3d_tool[1], point_3d_tool[2]],
                    "color": color,
                }
                # self.get_logger().info(f"block_id: {block_info['id']}")
                # self.get_logger().info(f"block_color: {block_info['color']}")

                new_blocks.append(block_info)
                self.target_count += 1  # 增加目标计数

        # 添加新检测到的方块
        self.detected_blocks.extend(new_blocks)

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def color_image_callback(self, msg):
        try:
            self.color_image_ = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {e}")

    def get_color_(self, roi) -> str:
        """根据ROI获取颜色"""
        if self.color_image_ is None:
            return "no_image"
        return get_color(self.color_image_, roi)

    def save_to_parameter_server(self):
        """将检测结果保存到参数服务器"""
        # 保存目标数量
        target_count_param = rclpy.Parameter(
            "target_count", rclpy.Parameter.Type.INTEGER, self.target_count
        )

        # 保存方块信息（序列化为JSON字符串）
        blocks_param = []
        for block in self.detected_blocks:
            block_data = {
                "id": block["id"],
                "tool_pos": block["tool_pos"],
                "color": block["color"],
            }
            blocks_param.append(block_data)

        self.detected_blocks_str = json.dumps(blocks_param)
        detected_blocks_param = rclpy.Parameter(
            "detected_blocks",
            rclpy.Parameter.Type.STRING,
            self.detected_blocks_str,
        )

        # 将所有参数放入一个列表中
        parameters_to_set = [
            target_count_param,
            detected_blocks_param,
        ]

        # 设置参数
        results = self.set_parameters(parameters_to_set)

        # 检查每个参数设置的结果
        for result in results:
            if not result.successful:
                self.get_logger().error(f"Failed to set parameter: {result.reason}")

        self.get_logger().info(
            f"Saved {len(self.detected_blocks)} blocks to parameter server."
        )
        return self.detected_blocks_str

    def is_new_block(self, position, min_distance=0.05):
        """检查是否是新方块（避免重复检测）"""
        for block in self.detected_blocks:
            existing_pos = block["tool_pos"]
            distance = np.sqrt(
                (position[0] - existing_pos[0]) ** 2
                + (position[1] - existing_pos[1]) ** 2
                + (position[2] - existing_pos[2]) ** 2
            )
            if distance < min_distance:
                return False
        return True

    def get_depth_value(self, x, y):
        """从深度图像获取深度值"""
        if self.depth_image is None:
            return None

        height, width = self.depth_image.shape
        if 0 <= x < width and 0 <= y < height:
            return self.depth_image[int(y), int(x)] / 1000.0  # 毫米转米
        return None

    def restart_detection_callback(self, request, response: Trigger.Response):
        """处理重启检测的请求"""
        """PS:
            ros2 service call /restart_detection std_srvs/srv/Trigger "{}"
        """
        self.get_logger().info("Restarting detection.")
        response.success = True
        response.message = self.save_to_parameter_server()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectionTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
