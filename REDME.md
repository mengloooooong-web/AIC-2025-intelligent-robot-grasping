开启llama大模型

```bash
llama-server -m ~/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf -c 2048 --threads 8 --port 8081
```

打开chat终端

```bash
ros2 run tools_demo chat
```

启动相机和识别节点

```bash
ros2 launch bringup dnn_node_start.launch.py
```

打开电脑浏览器输入板卡IP:8000

http://10.5.5.10:8000/

打开Connector和数据发布

```bash
ros2 launch bringup chat_bridge.launch.py 
```