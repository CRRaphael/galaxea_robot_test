#!/usr/bin/env python3
import logging
import os
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Union, Dict

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import json_numpy
import zmq

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

# Global cache for ROS data
cached_obs = {
    "full_image": None,       # numpy array (H,W,3), RGB
    "left_wrist_image": None, # numpy array (H,W,3), RGB
    "right_wrist_image": None,# numpy array (H,W,3), RGB
    "state": None,            # numpy array (1,14)
}

bridge = CvBridge()

def compressed_img_to_rgb_array(msg: CompressedImage) -> np.ndarray:
    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def callback_cam_head(msg):
    cached_obs["full_image"] = compressed_img_to_rgb_array(msg)

def callback_left_wrist(msg):
    cached_obs["left_wrist_image"] = compressed_img_to_rgb_array(msg)

def callback_right_wrist(msg):
    cached_obs["right_wrist_image"] = compressed_img_to_rgb_array(msg)

# These callbacks update parts of the state vector:
# state indices: 
# /hdas/feedback_gripper_left: index 7
# /hdas/feedback_gripper_right: index 14
# /hdas/feedback_arm_left: indices 1~6
# /hdas/feedback_arm_right: indices 8~13

# We'll keep a global numpy array of shape (14,) to update parts
global_state = np.zeros((14,), dtype=np.float32)

def callback_gripper_left(msg: Float64MultiArray):
    # Index 6 (0-based)
    if len(msg.data) > 0:
        global_state[6] = msg.data[0]

def callback_gripper_right(msg: Float64MultiArray):
    # Index 13 (0-based)
    if len(msg.data) > 0:
        global_state[13] = msg.data[0]

def callback_arm_left(msg: Float64MultiArray):
    # Assuming msg.data has length >= 6
    global_state[0:6] = msg.data[0:6]

def callback_arm_right(msg: Float64MultiArray):
    # Indices 7~12 (0-based)
    global_state[7:13] = msg.data[0:6]

def init_ros_subscribers():
    rospy.init_node('aloha_playback_client', anonymous=True)
    rospy.Subscriber("/hdas/camera_head/left_raw/image_raw_color/compressed", CompressedImage, callback_cam_head)
    rospy.Subscriber("/hdas/camera_wrist_left/color/image_raw/compressed", CompressedImage, callback_left_wrist)
    rospy.Subscriber("/hdas/camera_wrist_right/color/image_raw/compressed", CompressedImage, callback_right_wrist)
    rospy.Subscriber("/hdas/feedback_gripper_left", Float64MultiArray, callback_gripper_left)
    rospy.Subscriber("/hdas/feedback_gripper_right", Float64MultiArray, callback_gripper_right)
    rospy.Subscriber("/hdas/feedback_arm_left", Float64MultiArray, callback_arm_left)
    rospy.Subscriber("/hdas/feedback_arm_right", Float64MultiArray, callback_arm_right)

def log_message(message: str, log_file=None):
    print(message)
    logger.info(message)
    if log_file:
        log_file.write(message + "\n")
        log_file.flush()

@dataclass
class PlayConfig:
    playback_speed: float = 1.0
    trunk_size: int = 25
    loop_playback: bool = False
    local_log_dir: str = "./logs"
    run_id_note: Optional[str] = None
    seed: int = 7

def get_actions_from_server(observation, url="http://localhost:8777/act"):
    encoded = json_numpy.dumps(observation)
    response = requests.post(url, json={"encoded": encoded})
    if response.status_code != 200:
        raise RuntimeError(f"Server returned status {response.status_code}")
    actions = json_numpy.loads(response.content)
    return actions

def run_play(cfg: PlayConfig, log_file=None):
    init_ros_subscribers()

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    machine_ip = "192.168.2.147"
    socket.connect(f"tcp://{machine_ip}:5555")
    socket.RCVTIMEO = 60000  # 60s timeout

    rate_hz = 25 * cfg.playback_speed
    rate = rospy.Rate(rate_hz)

    step = 0

    while not rospy.is_shutdown():
        # 等待所有观测数据更新
        if None in [cached_obs["full_image"], cached_obs["left_wrist_image"], cached_obs["right_wrist_image"]]:
            log_message("Waiting for all images from ROS topics...")
            rate.sleep()
            continue

        cached_obs["state"] = global_state.copy().reshape(1,14)

        observation = {
            "instruction": "Open the drawer. place the toy into the drawer, and then close it",
            "full_image": cached_obs["full_image"],
            "left_wrist_image": cached_obs["left_wrist_image"],
            "right_wrist_image": cached_obs["right_wrist_image"],
            "state": cached_obs["state"],
        }

        try:
            actions = get_actions_from_server(observation)
        except Exception as e:
            logger.error(f"Failed to get actions from server: {e}")
            rate.sleep()
            continue

        # 发送动作给机器人
        try:
            socket.send_pyobj({
                'command': 'EXECUTE_ACTIONS',
                'actions': actions.tolist() if hasattr(actions, 'tolist') else actions,
            })
            response = socket.recv_string()
            if response == "ACK":
                log_message(f"Step {step}: Actions executed successfully", log_file=log_file)
            else:
                logger.error(f"Robot error response: {response}")
                break
        except Exception as e:
            logger.error(f"Error sending actions to robot: {e}")
            break

        step += 1
        rate.sleep()

    socket.close()
    context.term()

def setup_logging(cfg: PlayConfig):
    os.makedirs(cfg.local_log_dir, exist_ok=True)
    run_id = f"PLAYBACK-ROS"
    if cfg.run_id_note:
        run_id += f"--{cfg.run_id_note}"
    log_filepath = os.path.join(cfg.local_log_dir, run_id + ".txt")
    log_file = open(log_filepath, "w")
    logger.info(f"Logging to file: {log_filepath}")
    return log_file, log_filepath

def play_r1lite(cfg: PlayConfig):
    log_file, _ = setup_logging(cfg)
    try:
        run_play(cfg, log_file)
    finally:
        if log_file:
            log_file.close()

if __name__ == "__main__":
    time.sleep(2)  # 等待ROS话题准备
    cfg = PlayConfig()
    play_r1lite(cfg)