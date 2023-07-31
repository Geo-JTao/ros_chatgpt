from typing import List
import openai
import re
import argparse
import os
import json
import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT


class GPTAssistant:
    """
        加载ChatGPT配置和自定义预提示
    """
    def __init__(self) -> None:
        print("Initializing ChatGPT...")

        dir_path = os.path.dirname(__file__)
        parser = argparse.ArgumentParser()
        parser.add_argument("--openai_key", type=str, default=os.path.join(dir_path, "config.json"))
        parser.add_argument("--pri_prompt", type=str, default=os.path.join(dir_path, "prompts/primitives.txt"))
        parser.add_argument("--sce_prompt", type=str, default=os.path.join(dir_path, "prompts/scene.txt"))
        parser.add_argument("--sys_prompt", type=str, default=os.path.join(dir_path, "system_prompts/system.txt"))
        args = parser.parse_args()

        with open(args.openai_key, "r") as f:
            config = json.load(f)
        openai.api_key = config["OPENAI_API_KEY"]

        with open(args.sys_prompt, "r") as f:
            sys_prompt = f.read()

        with open(args.sce_prompt, "r") as f:
            sce_prompt = f.read()
        
        with open(args.pri_prompt, "r") as f:
            prompt = f.read()
        # chat_history列表，用于保存对话历史记录。初始对话历史包括系统预设提示、场景预设提示、用户输入和助手生成的示例代码。
        # 示例代码是用来演示如何打开机器人夹爪的。
        self.chat_history = [
            {
                "role": "system",
                "content": sys_prompt
            },
            {
                "role": "system",
                "content": sce_prompt
            },
            {
                "role": "user",
                "content": "open robotic gripper"
            },
            {
                "role": "assistant",
                "content": """```python
imi.openGripper(imi.target_gripper_angle)
```

This code uses the `openGripper(imi.target_gripper_angle)` function to open the gripper to a target angle from the current angle."""
            }
        ]

        self.ask(prompt)

        print(f"Done.")
        print(self.colors.BLUE + "Welcome to the SimBot! I am ready to help you with your questions and commands." + self.colors.ENDC)

    def ask(self, prompt):
        self.chat_history.append(
            {
                "role": "user",
                "content": prompt,
            }
        )
        # 客户端将输入的prompt消息发送给chatgpt，得到chatgpt全部的回复内容后发送给服务端
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.chat_history,
            temperature=0,
        )
        self.chat_history.append(
            {
                "role": "assistant",
                "content": completion.choices[0].message.content,
            }
        )
        return self.chat_history[-1]["content"]
    # 内部类colors，其中包含几个ANSI转义码，用于在终端中显示彩色文本。
    class colors: 
        RED = "\033[31m"
        ENDC = "\033[m"
        GREEN = "\033[32m"
        YELLOW = "\033[33m"
        BLUE = "\033[34m"


class GPTClient(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("%s already." % node_name)
        self.gpt_client = self.create_client(GPT, "gpt_service")
        while not self.gpt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for the server to go online...')
    
    def result_callback(self, result):
        response = result.result()
    
    def send_msg(self, msg):
        """
            Send messages to server.
        """
        request = GPT.Request()
        request.data = msg
        self.gpt_client.call_async(request).add_done_callback(self.result_callback)


def main(args=None):
    rclpy.init(args=args)
    node = GPTClient("gpt_client")
    gpt = GPTAssistant()

    while rclpy.ok():
        question = input(gpt.colors.YELLOW + "Assistant> " + gpt.colors.ENDC)

        if question == "!quit" or question == "!exit":
            break

        if question == "!clear":
            os.system("clear")
            continue

        response = gpt.ask(question)

        print(f"\n{response}\n")
        node.send_msg(response)
    
    node.destroy_node()
    rclpy.shutdown()
