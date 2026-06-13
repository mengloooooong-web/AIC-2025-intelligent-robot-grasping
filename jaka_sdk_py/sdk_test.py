import jkrc

robot = jkrc.RC("10.5.5.100")  # 返回一个机器人对象
ret = robot.login()  # 登录
if ret[0] == 0:
    print("登录成功")
else:
    print(ret)
    print("登录失败")
    robot.logout()  # 登出
    exit()

robot.logout()  # 登出
##--------------------------------
# IO_CABINET = 0  # 控制柜面板IO
# IO_TOOL = 1  # 工具IO
# IO_EXTEND = 2  # 扩展IO

# DO1 = 0  # 工具供电开关
# DO2 = 1  # 工具使能开关
# from time import sleep
# robot.power_on()  # 上电
# robot.set_digital_output(IO_TOOL, DO1, 1)  # 工具供电
# robot.set_digital_output(IO_TOOL, DO2, 1)  # 工具使能
# sleep(2)
# robot.set_digital_output(IO_TOOL, DO2, 0)  # 工具断开使能
# sleep(2)
# robot.set_digital_output(IO_TOOL, DO2, 1)  # 工具使能
# sleep(2)
# robot.set_digital_output(IO_TOOL, DO2, 0)  # 工具断开使能
# robot.set_digital_output(IO_TOOL, DO1, 0)  # 工具断开供电
# robot.power_off()  # 断电
# robot.logout()  # 登出

##--------------------------------
# 关节伺服运动
# ABS = 0  # 绝对运动
# INCR = 1  # 增量运动
# Enable = True
# Disable = False
# robot.enable_robot()
# robot.servo_move_enable(Disable)  # 退出位置控制模式
# robot.servo_move_enable(Enable)  # 进入位置控制模式
# print("enable")
# for i in range(200):
#     robot.servo_j(joint_pos=[0.00, 0, 0, 0, 0, 0.002], move_mode=INCR, step_num=10)  #
# for i in range(200):
#     robot.servo_j(joint_pos=[-0.00, 0, 0, 0, 0, -0.002], move_mode=INCR, step_num=10)
# robot.servo_move_enable(Disable)  # 退出位置控制模式
# print("disable")
# robot.logout()  # 登出

##--------------------------------
# 笛卡尔空间伺服运动
# PI = 3.14
# ABS = 0  # 绝对运动
# INCR = 1  # 增量运动
# Enable = True
# Disable = False

# robot.power_on()  # 上电
# robot.enable_robot()  # 使能
# joint_pos = [0.0, -PI / 4, -PI / 3, 0.0, -PI / 2.5, 0.0]
# # joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# robot.joint_move(joint_pos, ABS, True, 1)
# robot.servo_move_enable(Enable)  # 进入位置控制模式
# print("enable")
# for i in range(200):
#     robot.servo_p(end_pos=[0, 0, 0.1, 0, 0, 0], move_mode=INCR, step_num=1)
# for i in range(200):
#     robot.servo_p(end_pos=[0, 0, -0.1, 0, 0, 0], move_mode=INCR, step_num=1)
# robot.servo_move_enable(Disable)  # 退出位置控制模式
# print("disable")
# robot.logout()  #

##--------------------------------
# 末端直线运动
# from time import sleep  # noqa: E402

# # 运动模式
# ABS = 0
# INCR = 1
# tcp_pos = [0, 0, -30, 0, 0, 0]  # [x, y, z, rx, ry, rz]
# robot.power_on()  # 上电
# robot.enable_robot()
# print("move1")
# # 阻塞 沿z轴负方向 以10mm/s 运动30mm
# ret = robot.linear_move(tcp_pos, INCR, True, 10)
# print(ret[0])
# sleep(3)
# robot.logout()
##--------------------------------
# # 求机器人正解
# ret = robot.get_joint_position()
# if ret[0] == 0:
#     print("the joint position is :", ret[1])
# else:
#     print("some things happend,the errcode is: ", ret[0])
#     robot.logout()  # 登出
#     exit()
# joint_pos = ret[1]
# ret, cartesian_pose = robot.kine_forward(joint_pos)  # 求机器人正解
# if ret == 0:
#     print("the cartesian pose is :", cartesian_pose)
# else:
#     print("some things happend,the errcode is: ", ret)

# robot.logout()  # 登出
##--------------------------------
## 求机器人逆解 和 关节运动
#  x_position:=0.35 -p y_position:=0.01 -p z_position:=0.130 -p rx:=3.14159 -p ry:=0.0 -p rz:=-3.141592
# import math  # noqa: E402

# ret, ref_pos = robot.get_joint_position()
# if ret == 0:
#     print("the joint position is :", ref_pos)
# else:
#     print("some things happend,the errcode is: ", ret)
#     robot.logout()  # 登出
#     exit()
# cartesian_pose = [50.0, 220.4, 222.3, math.radians(180), 0.0, math.radians(-92.171)]
# # cartesian_pose = [350.0, 10.0, 130.0, 3.14159, 0.0, -3.141592]
# ret, joint_pos = robot.kine_inverse(ref_pos, cartesian_pose)  # 求机器人逆解
# if ret == 0:
#     print("the joint pose is :", joint_pos)
# else:
#     print("some things happend,the errcode is: ", ret)
#     robot.logout()
#     exit()

# # 机器人关节运动到目标点位
# # 运动模式
# ABS = 0
# INCR = 1
# # 阻塞模式
# is_block = True
# # 运动速度rad/s
# speed = 0.5
# robot.joint_move(joint_pos, ABS, is_block, speed)

# robot.logout()  # 登出
##--------------------------------
