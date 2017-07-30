# ROS Summer School 2017 挑战赛

## 团队介绍
* 队名： STAR Robot
* 单位： 上海科技大学自动化与机器人中心
* 成员： 陈宏宇, 旷皓飞, 龙肖灵

## 实验环境
* 硬件平台：
  * Turtlebot 2(Kinect相机需要面向地面安装)
  * PC(i7处理器， 8G内存)(PC的性能影响图像处理的效率)


* 软件环境：
  * Ubuntu 14.04
  * ROS Indigo
  * Python 2.7, OpenCV2.4.8

## Python以及OpenCV环境配置
* Python环境：Ubuntu 14.04 已预装了Python2.7


* 安装OpenCV：
  1. 软件源安装：
  ```
  sudo apt-get install python-numpy
  sudo apt-get install python-opencv
  ```
  也可使用pip或anaconda安装。
  2. 源码安装：到[OpenCV官网](http://opencv.org/)下载所需的版本， 按照其教程来安装。


* IDE配置：
  1. PyCharm：到[PyCharm官网](https://www.jetbrains.com/pycharm/)下载，按照其教程来安装
  2. 配置PyCharm：详见[ROS wiki IDEs](http://wiki.ros.org/IDEs)

## 算法简介
### Step 1 导入相机数据
* 读取相机数据：
  通过rospy订阅 '/camera/rgb/image_raw' 这一Topic：
  `self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)`
* 在回调函数里，利用cv_bridge将每一帧图像转换为opencv格式的图片：
  `image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')`
### Step 2 图像预处理
* 将原始图像转换为hsv图像
* 提取所需的颜色，将图像转换为二值图像
* 利用中值滤波和腐蚀来去除噪点
* 将处理后的图像切片，只保留相机下方某一范围内的区域
### Step 3 提取关键点
* 对Step 2中产生的二值图像进行轮廓提取：
  * 提取图片中的所有轮廓
  * 将轮廓面积小于一定值的轮廓去除(消除了某些滤波没有清除的噪声的影响)
  * 选取剩下的轮廓中面积最大的轮廓作为最终区域
* 经过上一步处理后，如果存在轮廓，取轮廓的质心作为最终所追踪的关键点
### Step 4 控制
* 通过对 '/cmd_vel_mux/input/teleop' Topic发送Twist类型的消息来控制小车移动
* 如果存在关键点：利用P控制器控制小车移动(线速度为定值，角速度受error影响，error为质心的x值减去图像中线的值(Kinect是320), error所乘的比例可以调节，其决定角速度的大小)
* 如果不存在关键点：
  在小车运动时，若是线离开了相机的视野(该处指对原图切片后的视野), 小车会立即停止运动，执行以下决策：
    1. 向某一方向旋转一定角度(根据最后一次error的正负来决定旋转的方向（error小于0，逆时针；error大于0，顺时针）)，若旋转过程中检测到了关键点(即线又回到了视野内)，则继续执行之前的决策; 若没有检测到，则执行2。
    2. 前进一段距离，停止(即到达终点或失败)。

## 参考资料
[Programming Robots with ROS: A Practical Introduction to the Robot Operating System](http://wiki.ros.org/Books/Programming_Robots_with_ROS)

我们对本书第12章中所提到的算法进行了改进，用于此次挑战赛

## 致谢
&emsp;&emsp;很感谢华东师范大学机器人运动与视觉实验室以及张新宇老师，还有暑期学校的组委会组织了此次活动。感谢GaiTech赞助了本次暑期学校的挑战赛。很高兴能与很多对机器人感兴趣的同学老师聚在一起交流学习。
&emsp;&emsp;由于时间较短，我们的程序还存在很多问题，如果同学老师对我们的程序有任何问题和建议，或是对我们的团队感兴趣，可以联系我们，我们的联系方式可以在我们实验室的主页上找到([上海科技大学自动化与机器人中心](https://robotics.shanghaitech.edu.cn/zh))
