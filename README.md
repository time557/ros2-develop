# ROS2-develop

RoboMaster-南京理工大学Alliance战队2026赛季培训使用

相关代码实现均在**ros2/src文件夹中**

任务一：简单的消息接收发布与日志输出
实现位置和功能包均在pub_sub_cmd文件中
主要包含publisher_function.cpp和subscriber_function.cpp用于生成发布节点和订阅节点
启动了两个节点后运行结果如下：
<img width="1345" height="423" alt="屏幕截图 2025-10-05 122007" src="https://github.com/user-attachments/assets/c0ba6f60-15b5-4a23-950e-64c32124443c" />
<img width="1340" height="425" alt="屏幕截图 2025-10-05 122013" src="https://github.com/user-attachments/assets/59f3c86f-c9af-4c4a-816e-0303ae7d1311" />



任务二：节点通信与数据可视化
相关功能包在data_visual文件中
包含pub_node.cpp和sub_node.cpp，分别生成产生方波、正弦信号的发生器，接收信号的订阅器并进行信号处理
使用foxglove与容器网络连接并数据可视化，结果如下：
<img width="1795" height="1178" alt="屏幕截图 2025-10-05 110843" src="https://github.com/user-attachments/assets/e9d80dc6-31e3-48fb-8903-b2408a47edf0" />

