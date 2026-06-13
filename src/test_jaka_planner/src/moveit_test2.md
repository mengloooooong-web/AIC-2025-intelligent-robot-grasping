输入目标笛卡尔坐标，规划并控制机械臂运动到目标位置。
ps:
ros2 run jaka_planner moveit_test_2 --ros-args -p model:=minicobo -p x_position:=-0.35 -p y_position:=0.01 -p z_position:=0.130 -p rx:=3.14159 -p ry:=0.0 -p rz:=0.0

ros2 run jaka_planner moveit_test_2 --ros-args -p model:=minicobo -p x_position:=0.35 -p y_position:=0.01 -p z_position:=0.130 -p rx:=3.14159 -p ry:=0.0 -p rz:=-3.141592

ros2 run jaka_planner moveit_test_2 --ros-args -p model:=minicobo -p x_position:=<x_position> -p y_position:=<y_position> -p z_position:=<z_position> -p rx:=<rx> -p ry:=<ry> -p rz:=<rz> -p use_metric:=true

默认单位为米，角度为弧度。use_metric为true时，单位为米，角度为弧度；use_metric为false时，单位为毫米，角度为度。
ps:
ros2 run jaka_planner moveit_test_2 --ros-args -p model:=minicobo -p x_position:=-350.88 -p y_position:=11.37 -p z_position:=130.0 -p rx:=180.0 -p ry:=0.0 -p rz:=0.0 -p use_metric:=false