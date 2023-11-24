# 第一次作业
```txt
1. 阅读ROSwiki关于发布者与订阅者的内容，云班课上传终端截图、代码。
https://wiki.ros.org/cn/ROS/TutorialsWritingPublisherSubscriber%28c%2B%2B%29
2. 完成ROSwiki关于创建ROS服务和客户端、消息和服务的实验，云班课上传终端截图、代码。
https://wiki.ros.org/cn/ROS/Tutorials/CreatingMsgAndSrv
https://wiki.ros.org/cn/ROS/TutorialsWritingServiceClient%28c%2B%2B%29
https://wiki.ros.org/cn/ROS/TutorialsExaminingServiceClient
3. 阅读ROSwiki Tutorials 的其他相关内容

# 第二次作业
```txt
写速度控制节点，实现机器人画圆运动，实验代码、视频上传云班课
1. 构建功能包vel，依赖roscpp rospy geometry_msgs
2. 创建新文件vel_node.cpp，新建节点vel_node
3. 申请发布话题/cmd_vel，创建发布对象vel_pub
4. 构建一个geometry_msgs/Twist类型的消息包vel_msg，用来承载要发送的速度值
5. 开启一个while循环，不停使用vel_pub对象发送速度消息包vel_msg
6. 连接底盘: roslaunch zoo_bringup bringup_with_imu.launch, rosrun启动速度节
点查看结果


编写程序获取激光雷达数据，实验代码、视频上传云班课：
1. 构建一个新的功能包lidar，依赖：roscpp rospy sensor_msgs
2. 在功能包中新建节点lidar_node
3. 在主函数中申请订阅话题/scan，并设置回调函数为LidarCallback()
4. 构建回调函数LidarCallback（），用来接收和处理雷达数据
5. 在回调函数调用ROS_INFO()显示雷达监测的前方障碍物距离