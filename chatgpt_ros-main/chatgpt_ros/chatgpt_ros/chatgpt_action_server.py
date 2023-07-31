import datetime
import os
import json
import urllib.request
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from chatgpt_ros_interfaces.action import Chat
from chatgpt_ros_interfaces.srv import SetPrompt, AddExample
from std_srvs.srv import Empty


class ChatBot:
    """
    Chatbot class for conversation.
    """

    def __init__(self, api_key, api_base, prompt=None):
        self.api_key = api_key
        self.api_base = api_base
        if prompt is None:
            current_date = datetime.datetime.now().strftime('%Y-%m-%d')
            self.prompt = f"You are ChatGPT, a large language model trained by OpenAI. Answer as concisely as possible. Knowledge cutoff: 2021-09-01, Current date: {current_date}."
        else:
            self.prompt = prompt
        self.reset()

    #将输入的事件流逐行解析，提取出有效的事件数据，并以生成器的形式逐个返回给调用方
    def _parse_stream(self, s):
        # 解析服务器发送的事件，s表示输入的流。在这里流被假设为可以迭代的对象，每次迭代返回一行文本
        for line in s:
            # 将当前行使用UTF-8编码进行解码，将字节串转换为字符串
            line = line.decode('utf-8')
            # 如果当前行以冒号开头，则忽略该行
            if line.startswith(':'):
                continue
            # 如果当前行去除首尾空格后等于 "data: [DONE]"，则返回 None。这表示事件流结束的标志。
            if line.strip() == "data: [DONE]":
                return None
            # 如果当前行以 "data: " 开头，则说明该行包含事件数据
            if line.startswith("data: "):
                # 解析从 "data: " 后开始的部分，并将其作为JSON字符串加载为Python对象。
                # 使用 yield 关键字返回加载后的对象。yield 使得该方法成为一个生成器（generator），允许按需逐个返回解析的事件数据
                yield json.loads(line[len("data: "):])


    # 向指定的 OpenAI API 地址发送请求，传递相应的参数和头部信息，并返回解析后的响应结果。
    def _request(self):
        # 创建一个urllib.request.Request对象表示一个 HTTP 请求
        req = urllib.request.Request(
            # 设置请求的 URL
            url=f'{self.api_base}/chat/completions',
            # 将包含请求参数的字典转换成 JSON 格式的字符串，并使用 UTF-8 编码转换为字节串。
            data=json.dumps({
                "model": "gpt-3.5-turbo",
                "temperature": 0,
                "stream": True,
                "messages": self.messages
            }).encode('utf-8'),
            # 设置请求头部信息，包括浏览器 User-Agent、Content-Type 和 Authorization 信息
            headers={
                "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36",
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}"
            }
        )
        # 使用 urllib.request.urlopen 方法发送请求，并解析
        return self._parse_stream(urllib.request.urlopen(req))

    # 与 ChatGPT 模型进行对话
    def chat(self, messages):
        self.messages.append({"role": "user", "content": messages})
        # 创建一个空列表 collected_messages，用于收集从请求中获取的消息
        collected_messages = []
        # 对每个响应分块进行迭代，其中每个块都包含了聊天模型的回复
        for chunk in self._request():
            # 从响应块中提取聊天模型的回复消息
            chunk_message = chunk['choices'][0]['delta']
            collected_messages.append(chunk_message)
            # 使用生成器 yield 返回回复消息中的内容部分。如果不存在内容，返回空字符串
            yield chunk_message.get('content', '')

        full_reply_role = ''.join([m.get('role', '') for m in collected_messages])
        full_reply_content = ''.join([m.get('content', '') for m in collected_messages])
        self.messages.append({"role": full_reply_role, "content": full_reply_content})
    
    # 获取最近一次聊天交互的内容
    def get_latest_content(self):
        return self.messages[-1]['content']
    
    def reset(self):
        self.messages = [
            {
                "role": "system",
                "content": self.prompt
            }
        ]


class ChatActionServer(Node):
    def __init__(self):
        # initialize node
        super().__init__('chatgpt_action_server')
        
        # load openai api key
        api_key = os.getenv("OPENAI_API_KEY")
        api_base = os.getenv("OPENAI_API_BASE", "https://api.openai.com/v1")
        assert api_key is not None, "OPENAI_API_KEY is not found in environment variables."

        # create chatbot object
        self._chatbot = ChatBot(api_key, api_base)
        
        # create ros action server
        self._action_server = ActionServer(self, Chat, 'chat', self._execute_callback)
        
        # Empty 服务类型可以用于触发简单的操作，而不需要传递参数或返回结果，用于重置 ChatGPT 模型的状态
        self._reset_service = self.create_service(Empty, 'reset', self._reset_callback)
        self._set_prompt_service = self.create_service(SetPrompt, 'set_prompt', self._set_prompt_callback)
        self._add_example_service = self.create_service(AddExample, 'add_example', self._add_example_callback)

    def _execute_callback(self, goal_handle):
        messages = goal_handle.request.messages
        self.get_logger().info(f'Chat goal received: {messages}')
        
        # stream chat result
        feedback_msg = Chat.Feedback()
        # 迭代生成器返回的每个聊天结果。
        for res in self._chatbot.chat(messages):
            feedback_msg.delta = res
            goal_handle.publish_feedback(feedback_msg)

        # chat finished
        goal_handle.succeed()

        # return total chat reply
        result = Chat.Result()
        result.content = self._chatbot.get_latest_content()
        self.get_logger().info('Chat finished.')
        return result

    def _reset_callback(self, request, response):
        """Reset chat history"""
        self._chatbot.reset()
        return response

    def _set_prompt_callback(self, request, response):
        """Set system prompt"""
        self._chatbot.prompt = request.prompt
        # 将其状态重置为初始状态。这样可以确保新的系统提示生效
        self._chatbot.reset()
        self.get_logger().info(f'Chat prompt changed: {self._chatbot.prompt}')
        return response

    # 向 ChatGPT 机器人添加对话示例
    def _add_example_callback(self, request, response):
        """Add example"""
        # 将用户和它的发言内容作为一个字典添加到ChatGPT的messages列表中
        self._chatbot.messages.append({"role": "user", "content": request.user})
        # 将机器人助手和它的回复内容作为一个字典添加到ChatGPT的messages列表中
        self._chatbot.messages.append({"role": "assistant", "content": request.assistant})
        return response


def main(args=None):
    rclpy.init(args=args)

    chat_action_server = ChatActionServer()

    rclpy.spin(chat_action_server)


if __name__ == '__main__':
    main()
