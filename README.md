# ChatGPT开源ROS软件包

## 项目网址
https://github.com/play-with-chatgpt/wpr_chatgpt/

## 使用步骤

1.  获取源码:
```
cd ~/catkin_ws/src/
git clone https://github.com/play-with-chatgpt/wpr_chatgpt.git
```
2. 安装依赖项:  
```
cd ~/catkin_ws/src/wpr_chatgpt/scripts
./install_deps.sh
```
3. 编译
```
cd ~/catkin_ws
catkin_make
```
4. 将OpenAI的API Key填写到wpr_chatgpt/config/api_key.yaml中。
5. 启动ChatGPT节点
ChatGPT接口：
```
roslaunch wpr_chatgpt chat.launch
```
GPT-3接口：
```
roslaunch wpr_chatgpt start.launch
```
6. 发送问题句子进行测试
```
rosrun wpr_chatgpt str_pub.py 你好
```

## 使用方法视频
视频网址：https://www.bilibili.com/video/BV1we4y1w73g

## ROS快速入门视频课
视频网址：https://www.bilibili.com/video/BV1BP4y1o7pw
![视频地址二维码](./media/ros1_course.png)
