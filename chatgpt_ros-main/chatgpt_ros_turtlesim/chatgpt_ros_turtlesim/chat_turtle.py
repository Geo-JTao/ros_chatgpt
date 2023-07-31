import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# SingleThreadedExecutor是一个执行器，用于在单个线程中执行ROS节点的回调函数，在单个线程中依次执行各个回调函数
from rclpy.executors import SingleThreadedExecutor
import threading
# Python 标准库中的正则表达式（regular expression）模块，匹配、查找和替换文本
import re
import math

from chatgpt_ros_interfaces.action import Chat
from chatgpt_ros_interfaces.srv import SetPrompt, AddExample
# SetPen 用于设置乌龟绘图时的画笔属性的服务，例如颜色、宽度等，相对位置、绝对位置对乌龟进行位移操作
from turtlesim.srv import SetPen, TeleportRelative, TeleportAbsolute
# 通过订阅乌龟的姿态信息，可以获取乌龟当前的位置和朝向。
from turtlesim.msg import Pose
from std_srvs.srv import Empty

# 系统提示符，初始化确定chatgpt所服务的目标定位、准则
system_prompt = """You are an assistant helping me with the simulator for robots.
When I ask you to do something, you are supposed to give me Python code that is needed to achieve that task using simulator and then 
an explanation of what that code does.
You are only allowed to use the functions I have defined for you.
You are not to use any other hypothetical functions that you think might exist.
You can use simple Python functions from libraries such as math and numpy."""

# 用户交互时候给chatgpt的提示符，包含具体的函数库、需求
user_prompt = """Here are some functions you can use to command the robot.

turtlebot.clear() - Delete the turtle's drawings from the screen. Do not move turtle. State and position of the turtle as well as drawings of other turtles are not affected.
turtlebot.forward(distance) - Move the turtle forward by the specified distance, in the direction the turtle is headed.
turtlebot.backward(distance) - Move the turtle backward by distance, opposite to the direction the turtle is headed. Do not change the turtle's heading.
turtlebot.right(angle) - Turn turtle right by degrees.
turtlebot.left(angle) - Turn turtle left by degrees.
turtlebot.setposition(x, y) - Move turtle to an absolute position. If the pen is down, draw line. Do not change the turtle's orientation.
turtlebot.setheading(to_angle) - Set the orientation of the turtle to to_angle.
turtlebot.home() - Move turtle to the origin - coordinates (0,0) - and set its heading to its start-orientation.
turtlebot.pendown() - Pull the pen down - drawing when moving.
turtlebot.penup() - Pull the pen up - no drawing when moving.
turtlebot.get_position() - Return the turtle's current location (x,y).
turtlebot.get_heading() - Return the turtle's current heading.

A few useful things: 
Instead of moveToPositionAsync() or moveToZAsync(), you should use the function setposition() that I have defined for you.
This is a two-dimensional environment, and setposition() accepts only 2 parameters.
Note that the robot is initially in the center of a square area of size 10*10, and you are not allowed to let the robot touch the boundary of the area."""

# 正确的交互示例，user是用户输入，assistant是chatgpt。给出的回答中有实际代码，可提取用于解决实际任务
examples = [
    {
        "user": "move 2 units forward",
        "assistant": """```python
turtlebot.forward(10)
```
This code uses the `forward()` function to move the robot to a new position that is 2 units before the current position."""
    }
]

# 在 Python 中，当定义一个不继承自其他类的简单类时，可以省略括号，变量名以一个下划线 _ 开头是一种命名约定，表示该变量是一个私有变量。
# 私有变量是指仅在类内部可访问和使用的变量，不能在类外部直接访问
class TurtleBot:
    def __init__(self, node, turtle_namespace='/turtle1'):
        self._node = node
        # 设置客户端订阅乌龟的绘图、相对位置、绝对位置、清屏服务；订阅乌龟的位姿
        self._set_pen_cli = node.create_client(SetPen, turtle_namespace + "/set_pen")
        self._teleport_relative_cli = node.create_client(TeleportRelative, turtle_namespace + "/teleport_relative")
        self._teleport_absolute_cli = node.create_client(TeleportAbsolute, turtle_namespace + "/teleport_absolute")
        self._clear_cli = node.create_client(Empty, "/clear")
        self._pose_sub = node.create_subscription(Pose, turtle_namespace + "/pose", self._pose_callback, 1)
        # 设置起始位姿，位于屏幕正中间
        self._origin = [5.544444561004639, 5.544444561004639, 0.0]
        self._pose = [0.0, 0.0, 0.0]
        
        self.home()
    
    def _pose_callback(self, msg):
        self._pose[0] = msg.x - self._origin[0]
        self._pose[1] = msg.y - self._origin[1]
        self._pose[2] = msg.theta - self._origin[2]
    
    def _degree_to_radian(self, angle):
        return angle * math.pi / 180.0
    
    '''
        rclpy.spin_until_future_complete()是一个阻塞方法,它会一直执行ROS的事件循环,直到传入的future完成。future 是一个表示异步操作结果的对象
        方法会在等待 future 完成时阻塞程序的执行，直到收到结果或超时。   
    '''
   
    def clear(self):
        # 从屏幕上删除乌龟的图画。不移动乌龟。海龟的状态和位置以及其他海龟的图纸不受影响。
        rclpy.spin_until_future_complete(self._node, self._clear_cli.call_async(Empty.Request()))
    
    def forward(self, distance):
        # 向前移动乌龟（int、float）
        req = TeleportRelative.Request()
        req.linear = float(distance)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
        
    def backward(self, distance):
        # 向后移动乌龟（int、float）
        req = TeleportRelative.Request()
        req.linear = -float(distance)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
    
    def right(self, angle):
        req = TeleportRelative.Request()
        req.angular = -self._degree_to_radian(angle)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
    
    def left(self, angle):
        req = TeleportRelative.Request()
        req.angular = self._degree_to_radian(angle)
        rclpy.spin_until_future_complete(self._node, self._teleport_relative_cli.call_async(req))
    
    def setposition(self, x, y):
        # 将海龟移动到一个绝对位置。如果笔在下面，就画一条线。不改变乌龟的方向
        req = TeleportAbsolute.Request()
        req.x = float(x) + self._origin[0]
        req.y = float(y) + self._origin[1]
        req.theta = self._pose[2]  + self._origin[2]
        rclpy.spin_until_future_complete(self._node, self._teleport_absolute_cli.call_async(req))
    
    def setheading(self, to_angle):
        # 设置乌龟的朝向
        req = TeleportAbsolute.Request()
        req.x = self._pose[0] + self._origin[0]
        req.y = self._pose[1] + self._origin[1]
        req.theta = to_angle + self._origin[2]
        rclpy.spin_until_future_complete(self._node, self._teleport_absolute_cli.call_async(req))
    
    def home(self):
        # 调用绝对位置服务使乌龟移动到起点
        req = TeleportAbsolute.Request()
        req.x = self._origin[0]
        req.y = self._origin[1]
        req.theta = self._origin[2]
        rclpy.spin_until_future_complete(self._node, self._teleport_absolute_cli.call_async(req))
    
    def pendown(self):
        # 放下画笔--移动时画线
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = 0
        rclpy.spin_until_future_complete(self._node, self._set_pen_cli.call_async(req))
    
    def penup(self):
        # 提起画笔--移动时不会画线
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = 1
        rclpy.spin_until_future_complete(self._node, self._set_pen_cli.call_async(req))
    
    def get_position(self):
        # Return the turtle's current location (x,y).
        return self._pose[:2]
    
    def get_heading(self):
        # Return the turtle's current heading.
        return self._pose[2]


class ChatTurlteClient(Node):
    def __init__(self):
        super().__init__('chat_action_client')
        self._action_client = ActionClient(self, Chat, 'chat')
        self._set_prompt_cli = self.create_client(SetPrompt, "set_prompt")
        self._add_example_cli = self.create_client(AddExample, "add_example")
    
    # 初始化，提供系统提示信息、示例数据和用户提示信息
    def initialize(self):
        req = SetPrompt.Request()
        req.prompt = system_prompt
        self._set_prompt_cli.wait_for_service()
        set_prompt_future = self._set_prompt_cli.call_async(req)
        rclpy.spin_until_future_complete(self, set_prompt_future)
        
        for ex in examples:
            req = AddExample.Request()
            req.user = ex['user']
            req.assistant = ex['assistant']
            self._add_example_cli.wait_for_service()
            add_example_future = self._add_example_cli.call_async(req)
            rclpy.spin_until_future_complete(self, add_example_future)
        
        self.send_goal(user_prompt, slient=True)
    
    ''' 与Chatgpt服务端的action通信过程 slient参数表示是否需要反馈''' 
    def send_goal(self, messages, slient=False):
        goal_msg = Chat.Goal()
        goal_msg.messages = messages

        self._action_client.wait_for_server()
        if slient:
            send_goal_future = self._action_client.send_goal_async(goal_msg)
        else:
            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        get_result_future = send_goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        if not slient:
            print()
        # 获取chatgpt的回复内容并返回
        return get_result_future.result().result.content

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # end='' 指定打印结束符为空，使打印的内容不换行。
        # flush=True 强制刷新输出，确保及时显示打印的内容
        print(feedback.delta, end='', flush=True)

# 提取chatgpt回复内容里的代码字符串
def extract_python_code(content):
    # 用于匹配包含在三个反引号''' '''之间的文本块、任意字符
    code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
    # 查找所有匹配的代码块
    code_blocks = code_block_regex.findall(content)
    if code_blocks:
        # 将所有代码块连接成一个完整的代码字符串
        full_code = "\n".join(code_blocks)
        # 检查完整的代码字符串是否以 "python" 开头，如果是，则去掉开头的 "python"
        if full_code.startswith("python"):
            full_code = full_code[7:]

        return full_code
    else:
        return None


def interact_function(node):
    turtlebot = TurtleBot(node)
    while rclpy.ok():
        # 提示用户输入信息，并将用户输入存储在 user_input 变量中
        user_input = input('user: ')
        reply = node.send_goal(user_input)
        code = extract_python_code(reply)
        if code is not None:
            try:
                exec(code)
            except Exception as e:
                node.send_goal(str(e))


def main(args=None):
    rclpy.init(args=args)

    node = ChatTurlteClient()
    node.initialize()
    
    interact_thread = threading.Thread(target=interact_function, args=(node,))
    # 启动交互线程,运行 interact_function
    interact_thread.run()
    
    executer = SingleThreadedExecutor()
    executer.add_node(node)
    executer.spin()


if __name__ == '__main__':
    main()
