cd ~/
git clone https://github.com/ggerganov/llama.cpp
cd llama.cpp
cmake -B build
cmake --build build --config Release

# llama.cpp
export PATH=/home/sunrise/llama.cpp/build/bin/:$PATH
 
 llama-cli -m qwen2.5-coder-0.5b-instruct-q4_k_m.gguf -n 512 -c 2048 -sys "你是编程小助手，我会告诉你要识别的物体颜色和编号，你按照{color:red,num:5}这样的格式回复；比如蓝色4号：{color:blue,num:4}" -co -cnv --threads 8
 
 