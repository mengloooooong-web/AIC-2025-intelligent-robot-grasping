"""
接收抓取请求
请求当前画面方块信息：
 - 工具坐标系位置
 - 颜色
执行：
 - 遍历方块信息->
    - 工具坐标系转换基坐标系
    - 颜色分组
 - 根据颜色分组，执行抓取->
    - 移动到指定位置
    - 吸取
    - 移动到放置位置
    - 释放
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import StrMsg
from std_srvs.srv import Trigger

# from custom_msgs.srv import MoveToPoint
import json

# import os
from time import sleep
from tools_demo.modules.jaka import Jaka
from tools_demo.modules.tools import tool_to_base

IO_CABINET = 0  # 控制柜面板IO
IO_TOOL = 1  # 工具IO
IO_EXTEND = 2  # 扩展IO

DO1 = 0  # 工具供电开关
DO2 = 1  # 工具使能开关

# 运动模式
ABS = 0  # 绝对位置
INCR = 1  # 增量位置


class Connecter(Node):
    def __init__(self):
        super().__init__("connecter")
        self.robot = Jaka()  # 返回一个机器人对象
        if not self.robot_is_ready():
            self.get_logger().error("robot is not ready")
            return

        # 放置区
        self.back_pose =[-100, 340, 200, 180.0, 0.0, 0.0]


        # 末端位置，拾取移动距离 单位mm
        self.end_pose_z = 155.0
        self.end_move_z = -8

        self.command = {}
        self.srv_ser = self.create_service(
            StrMsg, "send_command", self.connect_callback_
        )
        # ros2 service call /restart_detection std_srvs/srv/Trigger "{}"
        self.srv_client = self.create_client(Trigger, "restart_detection")
        self.target_count = 0
        self.blocks_data = []

    def robot_is_ready(self):
        ret = False
        if self.robot.login_state:
            self.get_logger().info("登录成功")
            self.robot.home_pose = [-252.7, -20.5, 293.7, 180, 0.0, 0.0]
            ret = self.robot.go_home()
            if ret[0] == 0:
                self.get_logger().info("move to point success")
                ret = True
            else:
                self.get_logger().error(f"some things happend,the errcode is:{ret}")
                ret = False
        else:
            self.get_logger().error("登录失败")
            ret = False
        self.get_logger().info("Connecter service started")
        return ret

    def restart_detection(self, done_cb=None):
        if not self.srv_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        self.get_logger().info("service available")

        request = Trigger.Request()
        future = self.srv_client.call_async(request)

        if done_cb:
            future.add_done_callback(done_cb)
            return future
        else:
            # 非阻塞轮询
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info("Service call completed")
            return future.result()
        
    
    def connect_callback_(self, request: StrMsg.Request, response: StrMsg.Response):
        """PS
        ros2 service call /send_command custom_msgs/srv/StrMsg "{data: '{\"color\": \"yellow\",\"num\": 3}'}"
     """
        self.get_logger().info(f"request received: {request.data}")
        try:
            # 预处理原始数据：去除首尾空白和方括号
            processed_data = request.data.strip().strip('[]')
            # 按 "],[" 分隔符分割多个命令块（兼容原逻辑的分隔方式）
            blocks = [block.strip().strip('[]{}') for block in processed_data.split('],') if block.strip()]
        
            # 提取第一个命令块
            if len(blocks) >= 1:
                first_block = blocks[0].strip()
            else:
                raise ValueError("No command blocks found in request data")
        
            # 提取第二个命令块
            second_block = None
            if len(blocks) >= 2:
                second_block = blocks[1].strip()
            else:
                self.get_logger().warn("Only one command block found, no second block available")
        
            #  分割第一个命令块的键值对
            key_value_pairs1 = []
            for pair in first_block.split(','):
                pair = pair.strip()
                if ':' in pair:
                    key, value = pair.split(':', 1)
                    key_value_pairs1.append((key.strip(), value.strip()))
        

            # 4. 处理第一个命令块的参数
            self.command = {}
            self.target_color = key_value_pairs1[0][1].strip('"')
        
            num1_found = None
            for key, value in key_value_pairs1[1:]:
                try:
                    num1_found = int(value.strip('"'))
                    break
                except ValueError:
                    continue
        
            if num1_found is None:
                try:
                    num1_found = int(key_value_pairs1[-1][1])
                except ValueError:
                    raise ValueError("No valid number found in first command block")
                
            # 2. 分割第二个命令块的键值对
            key_value_pairs2 = []
            for pair in second_block.split(','):
                pair = pair.strip()
                if ':' in pair:
                    key, value = pair.split(':', 1)
                    key_value_pairs2.append((key.strip(), value.strip()))


            # 4. 处理第二个命令块的参数
            #self.command = {}
            self.other_color= key_value_pairs2[0][1].strip('"')
        
            num2_found = None
            for key, value in key_value_pairs2[1:]:
                try:
                    num2_found = int(value.strip('"'))
                    break
                except ValueError:
                    continue
        
            if num2_found is None:
                try:
                    num2_found = int(key_value_pairs2[-1][1])
                except ValueError:
                    raise ValueError("No valid number found in second command block")

            self.target_num = num1_found
            self.other_num=num2_found

            self.get_logger().info(f"Processing first block: color={self.target_color}, num={self.target_num}")
            # 新增：打印第二个命令块（如果存在）
            if second_block:
                self.get_logger().info(f"Found second command block: {second_block}")
        
            # 启动检测
            self.restart_detection(done_cb=self.process_restart_result)
            response.success = True
            response.message = "Command received, restart_detection in progress"

        except Exception as e:
            self.get_logger().error(f"Error processing request: {str(e)}")
            response.success = False
            response.message = str(e)

        return response


    def process_restart_result(self, future):
        result: Trigger.Response = future.result()
        self.get_logger().info(f"Restart detection result: {result}")

        self.blocks_data = json.loads(result.message)
        current_pose = self.robot.get_tools_pos()

        if current_pose == -1:
            self.get_logger().error("Failed to get current tool position")
            return

        # 初始化两个列表：目标颜色和其他颜色
        target_list = []  # 目标颜色的方块
        other_list = []   # 其他颜色的方块

        # 转换坐标系并分类
        for item in self.blocks_data:
            pos = tool_to_base(item["tool_pos"], current_pose)
            color = item["color"]
            
            # 记录转换后的位置和颜色
            self.get_logger().info(f"item_pos: {item['tool_pos']}, pos: {pos}, color: {color}")
            
            # 分类存储
            if color == self.target_color:
                target_list.append((pos, color))
            else:
                other_list.append((pos, color))

        # 创建最终抓取列表（目标颜色在前，其他颜色在后）
        pos_lists = []
        # 添加目标颜色方块（最多self.target_num个）
        pos_lists.extend(target_list[:self.target_num])
        # 添加其他颜色方块（最多self.other_num个）
        pos_lists.extend(other_list[:self.other_num])


        # 执行抓取操作
        self.robot.pick_init()
        for pos, color in pos_lists:
            self.get_logger().info(f"Picking block of color: {color}")
            self.pick(pos[0], pos[1], pos[2])
        self.robot.pick_end()
        sleep(2)
        self.robot.go_home()

    def move_to_point(self, x, y, z):
        """
        先移动到指定位置,单位 mm
        """
        self.get_logger().info(f"move to point: {x}, {y}, {z}")
        ret = self.robot.go_point([x, y, z])
        if ret[0] == 0:
            self.get_logger().info("move to point success")
            return True
        else:
            self.get_logger().error(f"some things happend,the errcode is:{ret}")
            return False

    def pick(self, x, y, z):
        """
        先移动到指定位置,执行pick on操作,移动到目标位置,执行pick off操作
        """
        # 移动到指定位置
        offset_x =8.5
        if y>0:
            offset_y = -53
        else:
            offset_y = -53
        if self.move_to_point(
            x * 1000 + offset_x, y * 1000 + offset_y, self.end_pose_z
        ):
            sleep(1)
            self.robot.do_pick_on(self.end_move_z)
        else:
            return False
        # 移动到仓库位置
        sleep(1)
        if self.robot.go_pose(self.back_pose):
            # 吸盘松开
            self.robot.pick_off()
        else:
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    connecter = Connecter()
    try:
        rclpy.spin(connecter)
    except KeyboardInterrupt:
        pass
    connecter.get_logger().info("Shutdown")
    connecter.destroy_node()


if __name__ == "__main__":
    main()
