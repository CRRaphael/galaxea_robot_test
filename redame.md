# galaxea_robot_test

首先确认机器人位置比较安全，不要动起来的时候撞到桌子
长按电源开关启动机器人
确认电脑连接的无线网是iLearn5G

启动机器人之后，在电脑端连接机器人NUC：
ssh r1lite@192.168.2.147
机器人NUC密码是1

然后在NUC上跑启动所有机器人控制节点的脚本
cd ~/galaxea/install/share/startup_config/script/ 
./robot_startup.sh boot ../session.d/ATCStandard/R1LITEBody.d
可以tmux a查看具体情况（Ctrl B+W看所有节点，其中有几个报错是正常的，Ctrl B+D退出）
tmux kill-ser可以关闭所有节点

1. 如果要采集数据：
长按手柄侧开关至灯波动闪烁
电脑上连接手柄蓝牙，有......(L)和......(R)两个，连接成功后手柄灯变为常亮，为方便操作可以把手柄插到遥操作杆上
确认连接成功后，在电脑上跑启动遥操作节点的脚本
cd ~/R1_Lite_SDK/atc_host_standard-V1.1.9/install/share/startup_config/script/
./robot_startup.sh boot ../session.d/ATCHostStandard/R1LITET.d/
可以tmux a查看具体情况（Ctrl B+W看所有节点，其中有几个报错是正常的，Ctrl B+D退出）
tmux kill-ser可以关闭所有节点

顺利的话，稍等一会儿，机器臂则可以遥操作，手柄最上方的两个按钮控制夹爪开合，手柄遥感控制移动升降

然后运行我写的数据采集节点：
rosrun hdf5_collector data_collector_node.py
在里面选择路径之类的，输入s自动开始采集30秒，按q退出，采集结束会有文字提示，到时候再输入s开始下次采集

2. 如果要复播数据：
在NUC端跑：
cd /home/r1lite/galaxea/install/share/mobiman/scripts/motionControlTest/R1lite
python3 test.py

然后在电脑端跑：
conda activate action_test
cd ~/test
./eval.sh
配置文件之类的都在eval.sh里面改就行了
顺利的话，稍等一会儿，机身站直复位，机器臂则开始复播

主要的接收消息：
新窗口中rostopic echo XXXXXX可以查看数据信息，新窗口中rqt_image_view并在左上角选框里选择对应话题可以查看图像信息
各话题具体格式内容见https://docs.galaxea-ai.com/zh/Guide/R1Lite/R1Lite_Software_Introduction/#_9
"/hdas/camera_head/left_raw/image_raw_color/compressed",
"/hdas/camera_head/depth/depth_registered",
"/hdas/camera_wrist_left/color/image_raw/compressed",
"/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
"/hdas/camera_wrist_right/color/image_raw/compressed",
"/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
"/hdas/feedback_gripper_left",
"/hdas/feedback_gripper_right",
"/hdas/feedback_torso",
"/hdas/feedback_arm_left",
"/hdas/feedback_arm_right",
"/hdas/feedback_chassis"

踩坑：
1、机器人低电量：具体表现为NUC端的脚本莫名其妙自动Ctrl+C退出，充电就能解决。机器人充电较慢，最好平时多注意一些，提前充电
2、手柄低电量：具体表现为手柄打开很短时间就自动关闭或者和对机械臂的控制时好时坏，充电就能解决。
3、机器人线连接不良：具体表现为整机自检信息异常，机械臂莫名其妙不动，时好时坏，重插线解决。尤其需要注意的是机器人后方、紧贴NUC下方的那里有一处对接很容易松脱。
4、过大动作导致安全性停机：具体表现为机械臂悬在半空不动，关掉机器人重启就行了。特别注意关闭机器人时所有电机也关闭，悬在半空的机械臂会砸下来，一定要用力举着机械臂再关。
5、紧急制动开关未解锁：具体表现为什么都正常，但是机械臂和机身就是动不了，只有夹爪能动，把下面底盘上的紧急制动开关拧开就行。