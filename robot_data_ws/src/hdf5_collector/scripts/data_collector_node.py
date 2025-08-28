#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import h5py
import numpy as np
import cv2
from cv_bridge import CvBridge
import threading
import time
import os
import signal
import sys

# 导入ROS消息类型
from sensor_msgs.msg import CompressedImage, JointState

# # 导入 hdas_msg 包的 MotorControl 消息类型
# # 确保您的ROS工作空间已编译 hdas_msg 包，并且它在 PYTHONPATH 中
# try:
#     from hdas_msg.msg import motor_control as MotorControlMsg
# except ImportError:
#     rospy.logwarn("无法导入 'hdas_msg/motor_control'。将使用 'sensor_msgs/JointState' 作为替代。")
#     rospy.logwarn("如果您的ROS环境中已定义 'hdas_msg/motor_control'，请确保其已编译并位于 ROS_PACKAGE_PATH 中。")
#     # 如果 hdas_msg.msg.motor_control 不可用，则回退到 JointState
#     MotorControlMsg = JointState


class ROSDataCollector:
    """
    ROSDataCollector 类用于从ROS话题收集数据并保存到HDF5文件。
    它监听用户输入 's' 来启动一个指定时长的录制会话，并允许配置保存路径和采样频率。
    """
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ros_data_collector', anonymous=True)
        rospy.loginfo("ROS数据收集节点已初始化。")

        # CvBridge 用于图像转换
        self.bridge = CvBridge()

        # 数据缓冲区，用于在录制会话中收集数据
        self.image_cam_high_buffer = []
        self.image_cam_left_wrist_buffer = []
        self.image_cam_right_wrist_buffer = []
        self.effort_buffer = [] # 从 /hdas/feedback_arm_left/right 的 effort
        self.qpos_buffer = []   # 从 /hdas/feedback_arm_left/right 的 position
        self.qvel_buffer = []   # 从 /hdas/feedback_arm_left/right 的 velocity
        self.action_buffer = [] # 从 /motion_control/control_arm_left/right 的 p_des

        # 最新消息持有者，用于存储每个话题的最新接收消息
        # 'recording_loop' 将会从这些持有者中定期采样数据
        self.latest_cam_high_img = None
        self.latest_cam_left_wrist_img = None
        self.latest_cam_right_wrist_img = None
        self.latest_feedback_arm_left = None
        self.latest_feedback_arm_right = None
        self.latest_feedback_gripper_left = None
        self.latest_feedback_gripper_right = None
        self.latest_control_arm_left = None  # 新增：用于存储左臂控制指令
        self.latest_control_arm_right = None # 新增：用于存储右臂控制指令

        # 控制录制状态的标志和参数
        self.recording_active = False
        self.record_duration = 60.0  # 默认录制时长（秒）
        self.sampling_rate_hz = 10.0 # 默认采样频率（Hz）
        # 根据录制时长和采样频率计算预期的最大样本数
        self.max_samples = int(self.record_duration * self.sampling_rate_hz)
        self.save_directory = os.path.expanduser("/home/ubuntu/data_ft/2") 

        # 线程锁，用于在多线程访问数据缓冲区时确保线程安全
        self.data_lock = threading.Lock()

        # ROS订阅者：为每个所需的话题创建订阅者
        rospy.Subscriber("/hdas/camera_head/left_raw/image_raw_color/compressed", CompressedImage, self.cam_high_callback)
        rospy.Subscriber("/hdas/camera_wrist_left/color/image_raw/compressed", CompressedImage, self.cam_left_wrist_callback)
        rospy.Subscriber("/hdas/camera_wrist_right/color/image_raw/compressed", CompressedImage, self.cam_right_wrist_callback)
        rospy.Subscriber("/hdas/feedback_arm_left", JointState, self.feedback_arm_left_callback)
        rospy.Subscriber("/hdas/feedback_arm_right", JointState, self.feedback_arm_right_callback)
        rospy.Subscriber("/hdas/feedback_gripper_left", JointState, self.feedback_gripper_left_callback)
        rospy.Subscriber("/hdas/feedback_gripper_right", JointState, self.feedback_gripper_right_callback)
        # 新增订阅：用于获取 action 数据（p_des）
        # 现在使用导入的 MotorControlMsg 类型
        # rospy.Subscriber("/motion_control/control_arm_left", MotorControlMsg, self.control_arm_left_callback)
        # rospy.Subscriber("/motion_control/control_arm_right", MotorControlMsg, self.control_arm_right_callback)

        # 注册信号处理函数，以便在Ctrl+C时优雅退出
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        """
        处理Ctrl+C信号，确保程序优雅退出。
        """
        rospy.loginfo("检测到Ctrl+C，正在关闭数据收集器。")
        self.recording_active = False # 停止任何正在进行的录制
        rospy.signal_shutdown("用户中断。")
        sys.exit(0)

    def cam_high_callback(self, msg):
        """
        处理头部高分辨率相机（左眼）压缩图像话题的回调函数。
        将压缩图像解码为OpenCV图像格式（BGR），然后转换为RGB。
        """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.latest_cam_high_img = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logwarn(f"处理头部高分辨率相机图像回调时出错: {e}") # 使用warn避免大量错误日志

    def cam_left_wrist_callback(self, msg):
        """
        处理左手腕相机压缩图像话题的回调函数。
        将压缩图像解码为OpenCV图像格式（BGR），然后转换为RGB。
        """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.latest_cam_left_wrist_img = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logwarn(f"处理左手腕相机图像回调时出错: {e}")

    def cam_right_wrist_callback(self, msg):
        """
        处理右手腕相机压缩图像话题的回调函数。
        将压缩图像解码为OpenCV图像格式（BGR），然后转换为RGB。
        """
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.latest_cam_right_wrist_img = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logwarn(f"处理右手腕相机图像回调时出错: {e}")

    def feedback_arm_left_callback(self, msg):
        """
        处理左臂关节反馈话题的回调函数。
        """
        self.latest_feedback_arm_left = msg

    def feedback_arm_right_callback(self, msg):
        """
        处理右臂关节反馈话题的回调函数。
        """
        self.latest_feedback_arm_right = msg

    def feedback_gripper_left_callback(self, msg):
        """
        处理左夹爪行程反馈话题的回调函数。
        此数据目前未直接映射到HDF5的qpos/qvel/effort（14元素）字段，
        因为手臂反馈已包含7个关节（包括夹爪）。
        """
        self.latest_feedback_gripper_left = msg

    def feedback_gripper_right_callback(self, msg):
        """
        处理右夹爪行程反馈话题的回调函数。
        此数据目前未直接映射到HDF5的qpos/qvel/effort（14元素）字段。
        """
        self.latest_feedback_gripper_right = msg

    # def control_arm_left_callback(self, msg):
    #     """
    #     处理左臂控制指令话题的回调函数。
    #     现在直接使用 msg.p_des 字段。
    #     """
    #     self.latest_control_arm_left = msg

    # def control_arm_right_callback(self, msg):
    #     """
    #     处理右臂控制指令话题的回调函数。
    #     现在直接使用 msg.p_des 字段。
    #     """
    #     self.latest_control_arm_right = msg

    def _sample_data_and_append(self):
        """
        从最新的消息持有者中采样数据并追加到缓冲区。
        这个方法由 'recording_loop' 定期调用，以确保采样频率一致。
        """
        with self.data_lock:
            # 图像数据采样
            # 检查图像是否为空，若为空则使用零填充的占位符图像
            # 图像尺寸默认为480x640x3，类型为uint8
            current_cam_high_img = self.latest_cam_high_img.copy() if self.latest_cam_high_img is not None else np.zeros((480, 640, 3), dtype=np.uint8)
            current_cam_left_wrist_img = self.latest_cam_left_wrist_img.copy() if self.latest_cam_left_wrist_img is not None else np.zeros((480, 640, 3), dtype=np.uint8)
            current_cam_right_wrist_img = self.latest_cam_right_wrist_img.copy() if self.latest_cam_right_wrist_img is not None else np.zeros((480, 640, 3), dtype=np.uint8)

            self.image_cam_high_buffer.append(current_cam_high_img)
            self.image_cam_left_wrist_buffer.append(current_cam_left_wrist_img)
            self.image_cam_right_wrist_buffer.append(current_cam_right_wrist_img)

            # 关节状态数据采样 (qpos, qvel, effort)
            if self.latest_feedback_arm_left and self.latest_feedback_arm_right:
                # 获取左右臂的关节位置、速度和力矩，并确保长度至少为7。
                # 如果实际长度不足7，则用零填充以避免索引错误。
                qpos_left = np.array(self.latest_feedback_arm_left.position[:7], dtype=np.float32) if len(self.latest_feedback_arm_left.position) >= 7 else np.zeros(7, dtype=np.float32)
                qvel_left = np.array(self.latest_feedback_arm_left.velocity[:7], dtype=np.float32) if len(self.latest_feedback_arm_left.velocity) >= 7 else np.zeros(7, dtype=np.float32)
                effort_left = np.array(self.latest_feedback_arm_left.effort[:7], dtype=np.float32) if len(self.latest_feedback_arm_left.effort) >= 7 else np.zeros(7, dtype=np.float32)

                qpos_right = np.array(self.latest_feedback_arm_right.position[:7], dtype=np.float32) if len(self.latest_feedback_arm_right.position) >= 7 else np.zeros(7, dtype=np.float32)
                qvel_right = np.array(self.latest_feedback_arm_right.velocity[:7], dtype=np.float32) if len(self.latest_feedback_arm_right.velocity) >= 7 else np.zeros(7, dtype=np.float32)
                effort_right = np.array(self.latest_feedback_arm_right.effort[:7], dtype=np.float32) if len(self.latest_feedback_arm_right.effort) >= 7 else np.zeros(7, dtype=np.float32)

                # 将左右臂的数据连接起来，形成14元素的数组
                combined_qpos = np.concatenate((qpos_left, qpos_right))
                combined_qvel = np.concatenate((qvel_left, qvel_right))
                combined_effort = np.concatenate((effort_left, effort_right))

                self.qpos_buffer.append(combined_qpos)
                self.qvel_buffer.append(combined_qvel)
                self.effort_buffer.append(combined_effort)
                self.action_buffer.append(combined_qpos)
            else:
                # 如果左右臂反馈数据尚未全部可用，则追加零填充的占位符
                self.qpos_buffer.append(np.zeros(14, dtype=np.float32))
                self.qvel_buffer.append(np.zeros(14, dtype=np.float32))
                self.effort_buffer.append(np.zeros(14, dtype=np.float32))

            # # Action 数据采样 (p_des)
            # # 现在我们直接访问 MotorControlMsg 的 p_des 字段
            # if self.latest_control_arm_left and self.latest_control_arm_right:
            #     # 提取左臂和右臂的 p_des 字段
            #     # 并确保长度为7。
            #     action_left = np.array(self.latest_control_arm_left.p_des[:7], dtype=np.float32) if len(self.latest_control_arm_left.p_des) >= 7 else np.zeros(7, dtype=np.float32)
            #     action_right = np.array(self.latest_control_arm_right.p_des[:7], dtype=np.float32) if len(self.latest_control_arm_right.p_des) >= 7 else np.zeros(7, dtype=np.float32)
                
            #     # 连接左右臂的 p_des 形成 14 元素的 action 数组
            #     combined_action = np.concatenate((action_left, action_right))
            #     self.action_buffer.append(combined_action)
            # else:
            #     # 如果控制指令数据尚未全部可用，则追加零填充的占位符
            #     self.action_buffer.append(np.zeros(14, dtype=np.float32))

    def _clear_buffers(self):
        """
        清空所有数据缓冲区。在每次新的录制会话开始前调用。
        """
        with self.data_lock:
            self.image_cam_high_buffer = []
            self.image_cam_left_wrist_buffer = []
            self.image_cam_right_wrist_buffer = []
            self.effort_buffer = []
            self.qpos_buffer = []
            self.qvel_buffer = []
            self.action_buffer = [] # 清空 action 缓冲区
        rospy.loginfo("数据缓冲区已清空。")

    def _pad_or_truncate(self, data_list, target_length, item_shape, dtype):
        """
        根据目标长度对数据列表进行填充（用零）或截断。
        确保HDF5数据集具有固定形状。
        """
        current_length = len(data_list)
        if current_length == 0:
            # 如果列表为空，则返回一个填充了零的、目标长度和形状的数组
            # 确保item_shape是一个元组，即使是标量
            effective_item_shape = item_shape if isinstance(item_shape, tuple) else (item_shape,)
            return np.zeros((target_length,) + effective_item_shape, dtype=dtype)

        if current_length > target_length:
            # 如果数据量超过目标长度，则截断
            return np.array(data_list[:target_length], dtype=dtype)
        elif current_length < target_length:
            # 如果数据量不足目标长度，则填充零
            padding_needed = target_length - current_length
            effective_item_shape = item_shape if isinstance(item_shape, tuple) else (item_shape,)
            padding_shape = (padding_needed,) + effective_item_shape
            padding = np.zeros(padding_shape, dtype=dtype)
            return np.concatenate((np.array(data_list, dtype=dtype), padding), axis=0)
        else:
            # 如果数据量正好是目标长度，则直接转换为numpy数组
            return np.array(data_list, dtype=dtype)

    def save_to_hdf5(self):
        """
        将收集到的数据保存到HDF5文件。
        文件名包含时间戳，并保存在指定的目录下。
        """
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = f"robot_data_{timestamp}.hdf5"
        full_path = os.path.join(self.save_directory, filename)

        rospy.loginfo(f"正在将收集到的数据保存到 {full_path}...")
        try:
            with h5py.File(full_path, 'w') as f:
                # 创建顶层HDF5组
                observations_group = f.create_group('observations')
                images_group = observations_group.create_group('images')

                # 定义图像的默认形状和数据类型
                img_shape = (480, 640, 3)
                img_dtype = np.uint8
                if self.image_cam_high_buffer:
                    img_shape = self.image_cam_high_buffer[0].shape
                    img_dtype = self.image_cam_high_buffer[0].dtype

                # 处理并保存图像数据，确保数据集的形状为 (max_samples, 480, 640, 3)
                cam_high_data = self._pad_or_truncate(self.image_cam_high_buffer, self.max_samples, img_shape, img_dtype)
                cam_left_wrist_data = self._pad_or_truncate(self.image_cam_left_wrist_buffer, self.max_samples, img_shape, img_dtype)
                cam_right_wrist_data = self._pad_or_truncate(self.image_cam_right_wrist_buffer, self.max_samples, img_shape, img_dtype)

                images_group.create_dataset('cam_high', data=cam_high_data, dtype=img_dtype, compression="gzip")
                images_group.create_dataset('cam_left_wrist', data=cam_left_wrist_data, dtype=img_dtype, compression="gzip")
                images_group.create_dataset('cam_right_wrist', data=cam_right_wrist_data, dtype=img_dtype, compression="gzip")

                # 定义关节状态数据的形状和数据类型
                joint_state_shape = (14,) # 每个时间步的14个关节值
                joint_state_dtype = np.float32

                # 处理并保存关节状态数据 (effort, qpos, qvel)，确保数据集的形状为 (max_samples, 14)
                effort_data = self._pad_or_truncate(self.effort_buffer, self.max_samples, joint_state_shape, joint_state_dtype)
                qpos_data = self._pad_or_truncate(self.qpos_buffer, self.max_samples, joint_state_shape, joint_state_dtype)
                qvel_data = self._pad_or_truncate(self.qvel_buffer, self.max_samples, joint_state_shape, joint_state_dtype)

                observations_group.create_dataset('effort', data=effort_data, dtype=np.float32, compression="gzip")
                observations_group.create_dataset('qpos', data=qpos_data, dtype=np.float32, compression="gzip")
                observations_group.create_dataset('qvel', data=qvel_data, dtype=np.float32, compression="gzip")

                # 处理并保存 'action' 数据，它将由 control_arm_left 和 control_arm_right 的 p_des 组合而成
                action_data = self._pad_or_truncate(self.action_buffer, self.max_samples, joint_state_shape, joint_state_dtype)
                f.create_dataset('action', data=action_data, dtype=np.float32, compression="gzip")

                # 为 'base_action' 创建虚拟数据
                # 由于这些数据没有在订阅话题中指定来源，暂时使用零填充
                base_action_data = np.zeros((self.max_samples, 2), dtype=np.float32)
                f.create_dataset('base_action', data=base_action_data, dtype=np.float32, compression="gzip")

            rospy.loginfo(f"数据已成功保存到 {full_path}")
        except Exception as e:
            rospy.logerr(f"保存到HDF5时出错: {e}")

    def recording_loop(self):
        """
        录制循环：在指定时长内定期采样最新数据并将其添加到缓冲区。
        这个循环运行在一个单独的线程中，以避免阻塞主用户输入循环。
        """
        # 如果采样频率为0或负数，则避免除零错误和无限循环
        if self.sampling_rate_hz <= 0:
            rospy.logerr("采样频率必须是正数。请重新配置。")
            self.recording_active = False
            return

        rate = rospy.Rate(self.sampling_rate_hz)
        start_time = rospy.Time.now().to_sec()

        rospy.loginfo(f"开始录制，持续 {self.record_duration:.1f} 秒，采样频率为 {self.sampling_rate_hz:.2f} Hz...")
        
        # 确保循环在录制时长内进行，并且在ROS关闭时停止
        while self.recording_active and (rospy.Time.now().to_sec() - start_time) < self.record_duration and not rospy.is_shutdown():
            self._sample_data_and_append() # 采样数据
            try:
                rate.sleep() # 等待直到下一个采样周期
            except rospy.exceptions.ROSInterruptException:
                rospy.loginfo("录制循环被中断。")
                break # ROS节点关闭时退出循环

        self.recording_active = False # 录制结束
        rospy.loginfo("录制完成。")
        self.save_to_hdf5() # 保存数据到HDF5文件
        rospy.loginfo("准备好接收下一次 's' 输入。")

    def run(self):
        """
        主运行方法，处理用户输入以开始或停止数据收集。
        允许用户配置保存路径、录制时长和采样频率。
        """
        path_input = input(f"请输入HDF5文件保存路径 (默认为 {self.save_directory}/, 回车确认): ").strip()
        while not rospy.is_shutdown():
            try:
                # 获取保存目录
                while True:
                    if path_input:
                        expanded_path = os.path.expanduser(path_input)
                        if os.path.isdir(expanded_path):
                            self.save_directory = expanded_path
                            break
                        else:
                            try:
                                os.makedirs(expanded_path, exist_ok=True)
                                self.save_directory = expanded_path
                                rospy.loginfo(f"创建了目录: {self.save_directory}")
                                break
                            except OSError as e:
                                rospy.logerr(f"创建目录失败: {e}. 请输入有效路径。")
                    else:
                        break # 使用默认路径

                # # 获取录制时长
                # while True:
                #     duration_input = input(f"请输入录制总时长 (秒, 默认为 {self.record_duration} 秒): ").strip()
                #     if duration_input:
                #         try:
                #             duration = float(duration_input)
                #             if duration <= 0:
                #                 rospy.logwarn("录制时长必须大于0。")
                #             else:
                #                 self.record_duration = duration
                #                 break
                #         except ValueError:
                #             rospy.logwarn("无效输入。请输入一个数字。")
                #     else:
                #         break # 使用默认时长

                # # 获取采样频率
                # while True:
                #     rate_input = input(f"请输入采样频率 (Hz, 默认为 {self.sampling_rate_hz} Hz): ").strip()
                #     if rate_input:
                #         try:
                #             rate = float(rate_input)
                #             if rate <= 0:
                #                 rospy.logwarn("采样频率必须大于0。")
                #             else:
                #                 self.sampling_rate_hz = rate
                #                 break
                #         except ValueError:
                #             rospy.logwarn("无效输入。请输入一个数字。")
                #     else:
                #         break # 使用默认频率

                # 根据时长和频率计算最大样本数
                self.max_samples = int(self.record_duration * self.sampling_rate_hz)
                rospy.loginfo(f"当前配置：保存路径='{self.save_directory}', 时长={self.record_duration:.1f}s, 频率={self.sampling_rate_hz:.1f}Hz, 预计样本数={self.max_samples}。")
                print("\n") # 打印空行以分隔配置和操作提示

                user_action = input("按 's' 开始录制，按 'q' 退出程序: ").strip().lower()

                if user_action == 's':
                    if not self.recording_active:
                        self._clear_buffers() # 开始新录制前清空缓冲区
                        self.recording_active = True
                        # 在新线程中启动录制循环，不阻塞主线程
                        recording_thread = threading.Thread(target=self.recording_loop)
                        recording_thread.daemon = True # 设置为守护线程，主程序退出时自动终止
                        recording_thread.start()
                    else:
                        rospy.loginfo("录制已在进行中。")
                elif user_action == 'q':
                    rospy.loginfo("用户请求关闭数据收集器。")
                    break
                else:
                    rospy.loginfo("无效输入。请按 's' 或 'q'。")
            except EOFError:
                rospy.loginfo("输入结束，正在关闭数据收集器。")
                break
            except Exception as e:
                rospy.logerr(f"处理用户输入时发生错误: {e}")

        # 在主循环结束后，确保ROS节点正常关闭
        rospy.signal_shutdown("主循环退出。")
        # 保持节点运行，以便回调函数可以继续处理消息，直到完全关闭
        rospy.spin()

# 主程序入口
if __name__ == '__main__':
    collector = ROSDataCollector()
    collector.run()

