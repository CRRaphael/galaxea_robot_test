#!/usr/bin/env python

import sys

sys.path.append('/home/r1lite/galaxea/install/lib/python3/dist-packages')

import rospy
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from hdas_msg.msg import motor_control
import time
from std_msgs.msg import Bool
import zmq

from real_env import RealEnv


class JointStateSender:
    def __init__(self):
        rospy.init_node('joint_state_sender', anonymous=True)
        self.left_joint_state_pub = rospy.Publisher('/motion_target/target_joint_state_arm_left', JointState, queue_size=10)
        self.right_joint_state_pub = rospy.Publisher('/motion_target/target_joint_state_arm_right', JointState, queue_size=10)
        self.torso_joint_state_pub = rospy.Publisher('/torso_joint_target_position_', JointState, queue_size=10)
        self.torso_joint_state_pub_real = rospy.Publisher('/motion_target/target_joint_state_torso', JointState, queue_size=10)
        self.left_gripper_joint_state_pub = rospy.Publisher('/motion_target/target_position_gripper_left', JointState, queue_size=10)
        self.right_gripper_joint_state_pub = rospy.Publisher('/motion_target/target_position_gripper_right', JointState, queue_size=10)

        self.chassis_command_pub = rospy.Publisher('/motion_target/target_speed_chassis',TwistStamped, queue_size=10)
        self.acc_limit_pub = rospy.Publisher('/motion_target/chassis_acc_limit',TwistStamped,queue_size=10)
        self.breaking_mode_pub = rospy.Publisher('/motion_target/brake_mode',Bool,queue_size=10)
        self.command = None

        self.env = RealEnv(init_node=True)
    
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:5555")
        print("ZeroMQ server started at port: 5555")

        self.receiver_thread = threading.Thread(target=self.receive_actions)
        self.receiver_thread.daemon = True

        self.send_breaking_mode([1])
        self.send_vel_limit([1.6,1.6,1.6,4,4,4],[1.6,1.6,1.6,4,4,4], [1,1.6,1.4,1.4])

        rospy.on_shutdown(self.cleanup)

    def send_joint_state(self, position_left, position_right, position_torso):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub.publish(torso_joint_state)
    
    def send_gripper_joint_state(self, position_left, position_right):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right

        self.left_gripper_joint_state_pub.publish(left_joint_state)
        self.right_gripper_joint_state_pub.publish(right_joint_state)
        def send_joint_state_real(self, position_left, position_right, position_torso):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        right_joint_state = JointState()
        right_joint_state.position = position_right
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)
    def send_vel_limit(self, vel_limit_left, vel_limit_right, vel_limit_torso):
        left_joint_state = JointState()
        left_joint_state.velocity = vel_limit_left
        right_joint_state = JointState()
        right_joint_state.velocity = vel_limit_right
        torso_joint_state = JointState()
        torso_joint_state.velocity = vel_limit_torso

        self.left_joint_state_pub.publish(left_joint_state)
        self.right_joint_state_pub.publish(right_joint_state)
        self.torso_joint_state_pub_real.publish(torso_joint_state)

    def send_chassis(self, chassis_command):
        chassis_command_msg = TwistStamped()
        chassis_command_msg.header.stamp = rospy.Time.now()
        chassis_command_msg.linear.x = chassis_command[0]
        chassis_command_msg.linear.y = chassis_command[1]
        chassis_command_msg.angular.z = chassis_command[2]
        self.chassis_command_pub.publish(chassis_command_msg)

    def send_acc_limit(self,acc_limit):
        acc_limit_msg = TwistStamped()
        acc_limit_msg.header.stamp = rospy.Time.now()
        acc_limit_msg.linear.x = acc_limit[0]
        acc_limit_msg.linear.y = acc_limit[1]
        acc_limit_msg.angular.z = acc_limit[2]
        self.acc_limit_pub.publish(acc_limit_msg)

    def send_breaking_mode(self,breaking_mode_signal):
        breaking_mode_msg = Bool()
        breaking_mode_msg.data = breaking_mode_signal[0]
        self.breaking_mode_pub.publish(breaking_mode_msg)

    def reset(self):
        self.send_joint_state_real([0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0],[0, 0, 0, 0])
        time.sleep(1)

    def cleanup(self):
        rospy.loginfo("Cleaning up resources...")
        try:
            self.send_breaking_mode([0])
        except:
            rospy.logwarn("Failed to disable brake mode")
        try:
            self.socket.close()
            self.context.term()
        except:
            rospy.logwarn("Failed to close ZeroMQ resources")

    def send_14d_action(self, action):
        try:
            left_arm = action[0:7]
            right_arm = action[7:14]
            self.send_joint_state_real(
                left_arm[0:6], 
                right_arm[0:6],
                [0,0,0,0]
            )
            self.send_gripper_joint_state(
                [left_arm[6]], 
                [right_arm[6]]
            )

            return "success"
        except Exception as e:
            rospy.logerr(e)
            return "error"

    def receive_actions(self):
        while not rospy.is_shutdown():
            try:
                request = self.socket.recv_pyobj()

                if isinstance(request, dict) and request.get('command') == 'GET_OBS':
                    obs = self.env._get_obs()
                    self.socket.send_pyobj(obs)
                    print("Observation sent.")

                elif isinstance(request, dict) and request.get('command') == 'EXECUTE_ACTIONS':
                    action_sequence = request.get('actions')
                    print("Received action trunk")
                    cnt = 0
                    if isinstance(action_sequence, list) and all(len(a) == 14 for a in action_sequence):
                        for action in action_sequence:
                            stats = self.send_14d_action(action)
                            print(f'action{cnt+1} executed: {action} -- {stats}')
                            print(f'action{cnt+1} executed: {action}')
                            rospy.sleep(0.2)
                            cnt += 1
                        self.socket.send_string("ACK")
                    else:
                        rospy.logerr(f"Invalid action sequence format: {type(action_sequence)}")
                        self.socket.send_string("ERROR(INVALID_FORMAT)")

                elif isinstance(request, dict) and request.get('command') == 'RESET':
                    self.reset()
                    print('Reset Done.')
                    self.socket.send_string("Reset Done.")

                else:
                    rospy.logerr(f"Unknown request type: {request}")
                    self.socket.send_string("ERROR(UNKNOWN_REQUEST)")

            except Exception as e:
                rospy.logerr(f"Error processing actions: {str(e)}")

    def run(self):
        self.receiver_thread.start()
        rospy.spin()

if __name__ == '__main__':
    try:
        sender = JointStateSender()
        sender.run()
    except rospy.ROSInterruptException:
        pass



