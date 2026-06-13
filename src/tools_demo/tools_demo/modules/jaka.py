import jkrc
import math
from time import sleep

IO_CABINET = 0  # 控制柜面板IO
IO_TOOL = 1  # 工具IO
IO_EXTEND = 2  # 扩展IO

DO1 = 0  # 工具供电开关
DO2 = 1  # 工具使能开关

# 运动模式
ABS = 0  # 绝对位置
INCR = 1  # 增量位置


class Jaka:
    def __init__(self, ip="10.5.5.100"):
        self.ip_ = ip
        self.robot = jkrc.RC(ip)
        self.login_state = self.login()

        """
        [x, y, z, rx, ry, rz]单位:mm, deg
        """
        self.home_pose = [80.75, 266.5, 247.4, 180, 0.0, -92.171]

    def login(self):
        self.login_state = not self.robot.login()[0]
        self.robot.power_on()  # 上电
        sleep(1)
        self.robot.enable_robot()  # 使能机器人
        return self.login_state

    def go_home(self):
        """
        return:
            success:(0,)
            failed:other
        """
        ret = self.go_pose(self.home_pose)
        return ret

    def go_pose(self, pose):
        """
        末端笛卡尔坐标
        pose: [x, y, z, rx, ry, rz]
        单位:mm, deg
        return:
            success:(0,)
            failed:other
        """
        if not self.login_state:
            return (1, "not login")
        pose_ = [
            pose[0],
            pose[1],
            pose[2],
            math.radians(pose[3]),
            math.radians(pose[4]),
            math.radians(pose[5]),
        ]
        # 当前位置参考
        ret, ref_pos = self.robot.get_joint_position()
        # 逆解
        ret, joint_pos = self.robot.kine_inverse(ref_pos, pose_)
        # 绝对位置，阻塞模式，速度1rad/s
        ret = self.robot.joint_move(joint_pos, ABS, True, 1)
        return ret

    def pick_init(self):
        """
        供电打开
        """
        self.robot.set_digital_output(IO_TOOL, DO1, 1)

    def pick_on(self):
        """
        吸取
        """
        self.robot.set_digital_output(IO_TOOL, DO2, 0)

    def pick_off(self):
        """
        释放
        """
        self.robot.set_digital_output(IO_TOOL, DO2, 1)

    def pick_end(self):
        """
        夹爪关闭
        """
        self.robot.set_digital_output(IO_TOOL, DO1, 0)
        self.robot.set_digital_output(IO_TOOL, DO2, 0)

    def do_pick_on(self, move_z=-8, up_line_z=45):
        """
        向下直线运动并吸取，单位 mm
        move_z:向下运动距离
        up_line_z:吸取后抬起高度(>物体高度)
        """
        tcp_pos = [0, 0, move_z, 0, 0, 0]  # [x, y, z, rx, ry, rz]
        up_line = [0, 0, up_line_z, 0, 0, 0]  # [x, y, z, rx, ry, rz]
        # 阻塞 沿z轴负方向 以5mm/s 运动{end_move_z} mm
        self.robot.linear_move(tcp_pos, INCR, True, 5)
        self.pick_on()
        self.robot.linear_move(up_line, INCR, True, int(up_line_z / 2))

    def go_point(self, point):
        """
        末端笛卡尔坐标
        point: [x, y, z]
        单位:mm
        return:
            success:(0,)
            failed:other
        """
        pose_ = [point[0], point[1], point[2], math.radians(180), 0.0, 0.0]
        if not self.login_state:
            return (1, "not login")
        ret, ref_pos = self.robot.get_joint_position()
        # 逆解
        ret = self.robot.kine_inverse(ref_pos, pose_)
        if ret[0] != 0:
            return (1, "kine_inverse failed")
        # 绝对位置，阻塞模式，速度1rad/s
        ret = self.robot.joint_move(ret[1], ABS, True, 1)
        return ret

    def log_out(self):
        self.robot.disable_robot()
        self.robot.power_off()
        self.robot.logout()

    def get_tools_pos(self):
        """
        返回当前末端执行器的笛卡尔坐标
        return:
            success:[x, y, z, rx, ry, rz]单位:米, 弧度
            failed:-1
        """
        ret, pos = self.robot.get_tcp_position()
        if ret != 0:
            return -1
        # API返回单位为毫米，弧度
        # 转换为米，弧度
        pos = [pos[0] / 1000, pos[1] / 1000, pos[2] / 1000, pos[3], pos[4], pos[5]]
        return pos


robot = Jaka()
if __name__ == "__main__":
    print(robot.login())
    print(robot.go_home())
    sleep(1)
    # pose = [-216.7, 6.9, 285.0, 180, 0.0, 0.0]
    # pose = [80.75, 266.5, 247.4, 180, 0.0, -92.171]
    # pose = [-216.7, 6.9, 285.0, 180, 0.0, 0.0]
    # pose = [-212.7, 76.1, 293.7, 180, 0.0, 0.0]
    point = [-144.0, 343.2, 120.0]
    # print(robot.go_pose(pose))
    print(robot.go_point(point))
    # sleep(1)
    # print(robot.go_pose([80.75, 266.5, 247.4, 180, 0, 0]))
    # print(robot.go_point([80.75, 266.5, 247.4]))
    print(robot.get_tools_pos())
