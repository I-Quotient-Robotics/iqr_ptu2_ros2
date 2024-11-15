#!/usr/bin/python3


import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64, Int8
from iqr_ptu2 import PTU2


class PTU2Node(Node):

    def __init__(self) -> None:
        Node.__init__(self, node_name="ptu2")
        self.timer = self.create_timer(0.1, self.publish_callback)
        self.ptu2 = PTU2('/dev/pan_tilt')
        self.speed_publisher = self.create_publisher(Int8, '/ptu2/status/speed', 10)
        self.yaw_publisher = self.create_publisher(Float64, '/ptu2/status/yaw', 10)
        self.pitch_publisher = self.create_publisher(Float64, '/ptu2/status/pitch', 10)
        self.yaw_temp_publisher = self.create_publisher(Float64, '/ptu2/status/yaw_temp', 10)
        self.pitch_temp_publisher = self.create_publisher(Float64, '/ptu2/status/pitch_temp', 10)
        self.create_subscription(Int8, '/ptu2/goal/speed', self.subscribe_callback_speed_goal, 10)
        self.create_subscription(Float64, '/ptu2/goal/yaw', self.subscribe_callback_yaw_goal, 10)
        self.create_subscription(Float64, '/ptu2/goal/pitch', self.subscribe_callback_pitch_goal, 10)
        self.create_subscription(Int8, '/ptu2/cmd/speed', self.subscribe_callback_speed_cmd, 10)
        self.create_subscription(Float64, '/ptu2/cmd/yaw', self.subscribe_callback_yaw_cmd, 10)
        self.create_subscription(Float64, '/ptu2/cmd/pitch', self.subscribe_callback_pitch_cmd, 10)
    
    def subscribe_callback_speed_goal(self, goal:Int8) -> None:
        try: self.ptu2.speed = goal.data
        except: pass
    def subscribe_callback_yaw_goal(self, goal:Float64) -> None:
        try: self.ptu2.yaw = goal.data
        except: pass
    def subscribe_callback_pitch_goal(self, goal:Float64) -> None:
        try: self.ptu2.pitch = goal.data
        except: pass
    def subscribe_callback_speed_cmd(self, cmd:Int8) -> None:
        try: self.ptu2.speed += cmd.data
        except: pass
    def subscribe_callback_yaw_cmd(self, cmd:Float64) -> None:
        try: self.ptu2.yaw += cmd.data
        except: pass
    def subscribe_callback_pitch_cmd(self, cmd:Float64) -> None:
        try: self.ptu2.pitch += cmd.data
        except: pass

    def publish_callback(self):
        self.speed_publisher.publish(Int8(data=self.ptu2.speed))
        self.yaw_publisher.publish(Float64(data=self.ptu2.yaw))
        self.pitch_publisher.publish(Float64(data=self.ptu2.pitch))
        self.yaw_temp_publisher.publish(Float64(data=self.ptu2.yaw_temp))
        self.pitch_temp_publisher.publish(Float64(data=self.ptu2.pitch_temp))


if __name__ == '__main__':

    from sys import argv

    rclpy.init(args=argv)
    node = PTU2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()