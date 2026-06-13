import requests
# pip3 install requests
from custom_msgs.srv import StrMsg
import rclpy
from rclpy.node import Node


class LlamaChat(Node):
    """LlamaChat 类,
    需要启动
    llama-server -m /home/sunrise/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf -c 2048 --threads 8 --port 8081
    """

    def __init__(self, base_url="http://localhost:8081"):
        super().__init__("LlamaChat")
        self.base_url = base_url
        self.chat_history = []
        self.system_prompt = "你是编程小助手，我会告诉你要识别的物体颜色和数量，你按照[color:red,num:5]这样的格式回复;比如蓝色4个:[color:blue,num:4],如果包含多个颜色和数量，只解析第一个，后面的请忽略。请注意,你的回复必须严格遵循这个格式。不要回复其他内容。如中文、英文等。"

        self.send_command_service = self.create_client(StrMsg, "send_command")

        print("=== Llama 交互式聊天程序 ===")
        print("输入 'quit' 或 'exit' 退出程序")
        print("每次回复后可选择处理方式:")
        print("  N  - 不处理")
        print("  R  - 默认处理")
        print("-" * 40)
        while True:
            try:
                # 获取用户输入
                user_input = input("\n> ").strip()

                if user_input.lower() in ["quit", "exit"]:
                    print("再见！")
                    break

                if not user_input:
                    continue

                # 发送消息并获取回复
                print("正在思考...")
                response = self.send_message(user_input)
                print(f"\n助手: {response}")
                # 询问处理选项
                while True:
                    option = input("\n选择处理方式 [N/R]: ").strip()
                    if option.upper() in ["N", "R"]:
                        processed_response = self.process_response(response, option)
                        if processed_response != response:
                            print(f"处理结果: {processed_response}")
                        break
                    else:
                        print("请输入有效选项: N, R")

            except KeyboardInterrupt:
                print("\n\n程序被中断,再见！")
                break
            except Exception as e:
                print(f"发生错误: {str(e)}")

    def send_message(self, user_input):
        # 构建消息历史
        messages = [{"role": "system", "content": self.system_prompt}]
        messages.extend(self.chat_history)
        messages.append({"role": "user", "content": user_input})

        data = {
            "model": "qwen2.5-coder",
            "messages": messages,
            "max_tokens": 512,
            "temperature": 0.7,
            "stream": False,
        }

        try:
            response = requests.post(
                f"{self.base_url}/v1/chat/completions",
                json=data,
                headers={"Content-Type": "application/json"},
            )

            if response.status_code == 200:
                result = response.json()
                assistant_reply = result["choices"][0]["message"]["content"]

                # 更新对话历史
                self.chat_history.append({"role": "user", "content": user_input})
                self.chat_history.append(
                    {"role": "assistant", "content": assistant_reply}
                )

                return assistant_reply
            else:
                return f"错误: {response.status_code}, {response.text}"

        except Exception as e:
            return f"连接错误: {str(e)}"

    def process_response(self, response, option):
        """处理回复的不同选项"""
        if option.upper() == "N":
            print("不处理，继续对话...")
            return response
        elif option.upper() == "R":
            print("默认处理...")
            # 这里可以添加默认处理逻辑
            # 发布StrMsg到服务 send_command
            if not self.send_command_service.wait_for_service(10.0):
                print("服务 send_command 未准备好，无法发送命令")
                return response
            request = StrMsg.Request()
            request.data = response
            self.send_command_service.call_async(request)
            processed = f"[已处理] {response}"
            return processed
        else:
            print("未知选项，使用默认处理...")
            return response

def main(args=None):
    rclpy.init(args=args)
    chat = LlamaChat()
    chat.destroy_node()


if __name__ == "__main__":
    main()
