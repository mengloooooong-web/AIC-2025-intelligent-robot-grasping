# 智能机器人抓取系统 — 大模型任务挑战赛（产教融合赛）

## 📖 项目简介

本项目是一个基于 **ROS2** 的智能机器人抓取系统，深度融合**大语言模型（LLM）**与**计算机视觉**技术，实现了从自然语言指令到机械臂自主抓取的完整闭环。系统采用 LangGraph 智能体框架作为决策核心，结合 YOLOv8 实时目标检测与 JAKA 机械臂运动规划，能够自动理解用户描述（如"抓取蓝色的3号积木"），完成目标识别、位姿估计与抓取执行。

### 核心特性

- 🤖 **大模型自然语言交互**：基于 llama.cpp 部署 Qwen2.5-Coder 模型，将自然语言指令解析为结构化抓取任务
- 👁️ **YOLOv8 视觉检测**：实时检测积木目标的位置、颜色与编号，输出检测框与类别信息
- 🧠 **LangGraph 智能体调度**：以状态图方式编排抓取全流程（感知→决策→规划→执行→验证）
- 🦾 **JAKA 机械臂控制**：集成 MoveIt 2 运动规划框架，支持伺服级实时控制
- 📷 **深度相机感知**：接入 InuTech 深度相机，输出 RGB、深度图、点云、IMU 等多模态数据
- 🌐 **Web 可视化监控**：基于 WebSocket 的实时视频流展示与检测结果渲染

---

## 🏗️ 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                     用户自然语言指令                        │
│                e.g. "抓取蓝色的3号积木"                     │
└──────────────────────┬───────────────────────────────────┘
                       ▼
┌──────────────────────────────────────────────────────────┐
│               LLM 服务 (llama.cpp)                        │
│            Qwen2.5-Coder 0.5B Instruct                    │
│            格式化输出: {color:blue, num:3}                 │
└──────────────────────┬───────────────────────────────────┘
                       ▼
┌──────────────────────────────────────────────────────────┐
│              LangGraph Agent 智能体                        │
│     ┌─────┐   ┌─────┐   ┌──────┐   ┌──────┐   ┌────┐   │
│     │感知 │──▶│决策 │──▶│规划  │──▶│执行  │──▶│验证│   │
│     └─────┘   └─────┘   └──────┘   └──────┘   └────┘   │
└──────┬────────────────────┬──────────────────────────────┘
       ▼                    ▼
┌──────────────┐   ┌──────────────────┐
│ DNN 检测节点  │   │  JAKA 机械臂控制  │
│  YOLOv8      │   │  MoveIt 2 规划   │
│  dnn_node    │   │  servoj_demo     │
└──────┬───────┘   └──────────────────┘
       ▼
┌──────────────────────────────────────────────────────────┐
│              InuTech 深度相机 (inuros2)                    │
│   RGB / Depth / PointCloud / IMU / Fisheye               │
└──────────────────────────────────────────────────────────┘
```

---

## 📁 项目结构

```
.
├── README.md                          # 项目说明文档
├── chat_interactive.py                # LLM 交互式聊天客户端
├── llama_readme.txt                   # llama.cpp 部署说明
├── REDME.md                           # 快速启动命令参考
├── .gitignore
├── src/
│   ├── bringup/                       # 启动与配置包
│   │   ├── launch/
│   │   │   ├── dnn_node_start.launch.py    # YOLO检测节点启动
│   │   │   ├── cam_web.launch.py           # 相机+Web可视化启动
│   │   │   ├── chat_bridge.launch.py       # LLM通信桥接启动
│   │   │   ├── jaka_start.launch.py        # JAKA机械臂启动
│   │   │   ├── langgraph_agent.launch.py   # LangGraph智能体启动
│   │   │   ├── servoj_demo.launch.py       # 伺服控制演示
│   │   │   └── tf_pub.launch.py            # TF坐标发布
│   │   ├── config/
│   │   │   ├── best.pt                     # YOLOv8 最佳权重
│   │   │   ├── last.pt                     # YOLOv8 最后权重
│   │   │   ├── block.list                  # 检测类别列表
│   │   │   ├── yolov8nblockconfig.json     # YOLOv8模型配置
│   │   │   ├── yolo*.bin                   # YOLO推理模型文件
│   │   │   └── test_model.py               # 模型测试脚本
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── custom_msgs/                    # 自定义ROS2消息/服务
│   │   ├── msg/
│   │   │   └── Blocks.msg                  # 积木位姿消息
│   │   ├── srv/
│   │   │   ├── MoveToPoint.srv             # 移动到目标点服务
│   │   │   ├── RestartDetection.srv        # 重启检测服务
│   │   │   └── StrMsg.srv                  # 字符串消息服务
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── inuros2/                        # InuTech相机ROS2驱动
│       ├── include/                        # C++ 头文件
│       │   ├── ros_sensor.h                # 传感器管理
│       │   ├── ros_depth_publisher.h       # 深度图发布
│       │   ├── ros_pointcloud_publisher.h  # 点云发布
│       │   ├── ros_video_publisher.h       # 视频发布
│       │   ├── ros_imu_publisher.h         # IMU发布
│       │   ├── ros_features_publisher.h    # 特征发布
│       │   ├── ros_objectdetection_publisher.h # 目标检测发布
│       │   └── ...                         # 其他模块
│       ├── src/                            # C++ 源文件
│       │   ├── filter/                     # 滤波器（深度→点云/空间/时域）
│       │   └── *.cpp                       # 各模块实现
│       ├── config/
│       │   └── params.yaml                 # 相机参数配置
│       ├── msg/
│       │   └── Features.msg                # 特征消息定义
│       ├── launch/                         # 相机启动文件
│       ├── CMakeLists.txt
│       └── build.sh                        # 编译脚本
```

---

## 🚀 快速开始

### 环境要求

| 组件 | 版本/说明 |
|------|----------|
| 操作系统 | Ubuntu 22.04 (Jammy) |
| ROS2 | Humble Hawksbill |
| C++ 标准 | C++17 |
| Python | ≥ 3.8 |
| 硬件平台 | D-Robotics RDK X5 开发板 |
| 深度相机 | InuTech 系列 (M4.5) |
| 机械臂 | JAKA MiniCobo |
| 推理框架 | dnn_node (地平线BPU) |

### 依赖安装

```bash
# ROS2 基础依赖
sudo apt install ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-pcl-conversions ros-humble-vision-msgs

# 编译工具
sudo apt install build-essential cmake git

# Python 依赖
pip install ultralytics opencv-python requests numpy
```

### 编译项目

```bash
cd ~/ros2_ws
colcon build --packages-select custom_msgs inuros2 bringup tools_demo
source install/setup.bash
```

### 1. 部署大语言模型（llama.cpp）

```bash
# 克隆并编译 llama.cpp
cd ~/
git clone https://github.com/ggerganov/llama.cpp
cd llama.cpp
cmake -B build
cmake --build build --config Release

# 下载 Qwen2.5-Coder 模型并启动服务
llama-server -m ~/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf \
  -c 2048 --threads 8 --port 8081 &
```

### 2. 启动相机与识别节点

```bash
ros2 launch bringup dnn_node_start.launch.py
```

### 3. 启动机械臂驱动

```bash
ros2 launch bringup jaka_start.launch.py
```

### 4. 启动 LangGraph 智能体

```bash
ros2 launch bringup langgraph_agent.launch.py
```

### 5. 启动 LLM 通信桥接

```bash
ros2 launch bringup chat_bridge.launch.py
```

### 6. 打开 Web 可视化

浏览器访问：`http://<板卡IP>:8000/`（默认 `10.5.5.10:8000`）

---

## 🎮 使用方法

### 交互式聊天终端

```bash
ros2 run tools_demo chat
```

输入自然语言指令，例如：
- "请抓取蓝色的3号积木"
- "把红色5号放到左边"
- "抓取黄色2号积木块"

系统自动解析为结构化命令并执行抓取流程。

### 通过 ROS2 Service 直接调用

```bash
# 发送抓取命令
ros2 service call /agent_send_command custom_msgs/srv/StrMsg \
  "{data: '{\"color\": \"yellow\", \"num\": 3}'}"

# 重启检测
ros2 service call /restart_detection custom_msgs/srv/RestartDetection \
  "{target_count: 5, detection_timeout: 30.0}"
```

---

## 🔧 关键配置

### YOLOv8 检测配置 (`yolov8nblockconfig.json`)

```json
{
    "model_file": "config/yolov8n_block_detect_640x640_nv12.bin",
    "task_num": 4,
    "dnn_Parser": "yolov8",
    "class_num": 1,
    "score_threshold": 0.25,
    "nms_threshold": 0.7
}
```

### 相机参数 (`params.yaml`)

- **分辨率**: Binning 模式（640×400）
- **深度**: 通道3，原始深度格式
- **帧率**: 15 FPS
- **点云**: 支持深度图→点云转换

---

## 🧪 关键技术细节

### LLM Prompt 工程

系统使用结构化 Prompt 约束大模型输出格式：

```
你是编程小助手，我会告诉你要识别的物体颜色和编号，
你按照{color:red,num:5}这样的格式回复；
比如蓝色4号:{color:blue,num:4}，
没有显式指定颜色和编号时，默认颜色为none，编号为-1号{color:none,num:-1}。
请注意，你的回复必须严格遵循这个格式。
```

### LangGraph Agent 工作流

```
START → 解析指令 → 触发检测 → 获取检测结果 → 
匹配合适目标 → 计算抓取位姿 → 执行抓取 → 
验证结果 → END (或重试)
```

- 最大重试次数: 3（可配置 `agent_max_retries`）
- 检测置信度阈值: 0.55（可配置 `detection_threshold`）

### 自定义 ROS2 服务

| 服务名 | 类型 | 功能 |
|--------|------|------|
| `/agent_send_command` | `StrMsg` | 发送结构化抓取命令 |
| `/restart_detection` | `RestartDetection` | 重启目标检测 |
| `/move_to_point` | `MoveToPoint` | 机械臂点到点运动 |

---

> 本项目为「大模型任务挑战赛（产教融合赛）」参赛作品，感谢赛事组委会与各位指导老师的支持！
