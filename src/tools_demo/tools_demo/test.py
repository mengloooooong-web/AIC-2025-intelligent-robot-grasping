#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-发送两个加数，请求加法器计算
"""

# import sys

import rclpy  # ROS2 Python接口库
from rclpy.node import Node  # ROS2 节点类
from custom_msgs.srv import MoveToPoint  # 自定义的服务接口


class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化
        self.client = self.create_client(
            MoveToPoint, "move_to_point"
        )  # 创建服务客户端对象（服务接口类型，服务名）
        while not self.client.wait_for_service(
            timeout_sec=1.0
        ):  # 循环等待服务器端成功启动
            self.get_logger().info("service not available, waiting again...")
        self.request = MoveToPoint.Request()  # 创建服务请求的数据对象

    def send_request(self):  # 创建一个发送服务请求的函数
        # 0.2967, -0.1865, 0.1280
        self.request.point.x = 0.2967
        self.request.point.y = -0.1865
        self.request.point.z = 0.1280
        self.future = self.client.call_async(self.request)  # 异步方式发送服务请求


def main(args=None):
    rclpy.init(args=args)  # ROS2 Python接口初始化
    node = adderClient("service_adder_client")  # 创建ROS2节点对象并进行初始化
    node.send_request()  # 发送服务请求

    while rclpy.ok():  # ROS2系统正常运行
        rclpy.spin_once(node)  # 循环执行一次节点

        if node.future.done():  # 数据是否处理完成
            try:
                response = node.future.result()  # 接收服务器端的反馈数据
            except Exception as e:
                node.get_logger().info("Service call failed %r" % (e,))
            else:
                if response is None:
                    node.get_logger().info("Service call failed")
                else:
                    result: MoveToPoint.Response = response
                    if result.success:
                        node.get_logger().info("Service call succeeded")
                    else:
                        node.get_logger().info("Service call failed")
                    break

    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS2 Python接口
