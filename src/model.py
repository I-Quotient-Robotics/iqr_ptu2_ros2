#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from sys import argv
from math import radians


class JointStatePublisher(Node):
    def __init__(self):
        Node.__init__(self, 'ptu2_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_state = JointState()
        self.joint_state.name = ['ptu2_yaw_joint', 'ptu2_pitch_joint']
        self.__yaw = 0.0
        self.__pitch = 0.0
        self.create_subscription(Float64, '/ptu2/status/yaw', self.subscribe_callback_yaw_goal, 10)
        self.create_subscription(Float64, '/ptu2/status/pitch', self.subscribe_callback_pitch_goal, 10)
    
    def subscribe_callback_yaw_goal(self, status:Float64): self.__yaw = status.data
    def subscribe_callback_pitch_goal(self, status:Float64): self.__pitch = status.data

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.header.frame_id = 'base_link'
        self.joint_state.position = (radians(self.__yaw), radians(self.__pitch))
        self.publisher.publish(self.joint_state)


if __name__ == '__main__':

    rclpy.init(args=argv)

    joint_state_publisher = JointStatePublisher()

    rclpy.spin(joint_state_publisher)

    joint_state_publisher.destroy_node()

    rclpy.shutdown()
