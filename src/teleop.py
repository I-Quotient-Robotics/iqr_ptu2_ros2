#!/usr/bin/python3


import rclpy
from rclpy.node import Node
import rclpy.wait_for_message
from std_msgs.msg import Float64, Int8
from getch import getch


class PTU2Teleop(Node):

    def __init__(self) -> None:
        Node.__init__(self, node_name="ptu2_teleop")
        self.speed_cmd_publisher = self.create_publisher(Int8, '/ptu2/cmd/speed', 10)
        self.yaw_cmd_publisher = self.create_publisher(Float64, '/ptu2/cmd/yaw', 10)
        self.pitch_cmd_publisher = self.create_publisher(Float64, '/ptu2/cmd/pitch', 10)
        self.speed_goal_publisher = self.create_publisher(Int8, '/ptu2/goal/speed', 10)
        self.yaw_goal_publisher = self.create_publisher(Float64, '/ptu2/goal/yaw', 10)
        self.pitch_goal_publisher = self.create_publisher(Float64, '/ptu2/goal/pitch', 10)
        self.__step = 2

    def step_up(self) -> None: self.__step+=1
    def step_down(self) -> None: self.__step-=1
    def speed_up(self) -> None: self.speed_cmd_publisher.publish(Int8(data=1*self.__step))
    def speed_down(self) -> None: self.speed_cmd_publisher.publish(Int8(data=-1*self.__step))
    def left(self) -> None: self.yaw_cmd_publisher.publish(Float64(data=2.0*self.__step))
    def right(self) -> None: self.yaw_cmd_publisher.publish(Float64(data=-2.0*self.__step))
    def forward(self) -> None: self.pitch_cmd_publisher.publish(Float64(data=2.0*self.__step))
    def back(self) -> None: self.pitch_cmd_publisher.publish(Float64(data=-2.0*self.__step))
    def reset(self) -> None:
        self.__step = 2
        self.speed_goal_publisher.publish(Int8(data=10))
        self.yaw_goal_publisher.publish(Float64(data=0.0))
        self.pitch_goal_publisher.publish(Float64(data=0.0))


if __name__ == '__main__':

    from sys import argv, exit
    from signal import signal, SIGINT

    rclpy.init(args=argv)
    def ctrl_c_exit(*_):
        rclpy.shutdown()
        exit(0)
    signal(SIGINT, ctrl_c_exit)

    ptu2_teleop = PTU2Teleop()

    key_map = {
        b'R': ptu2_teleop.step_up,    b'r': ptu2_teleop.step_up,
        b'F': ptu2_teleop.step_down,  b'f': ptu2_teleop.step_down,
        b'Q': ptu2_teleop.speed_up,   b'q': ptu2_teleop.speed_up,
        b'E': ptu2_teleop.speed_down, b'e': ptu2_teleop.speed_down,
        b'W': ptu2_teleop.forward,    b'w': ptu2_teleop.forward,
        b'S': ptu2_teleop.back,       b's': ptu2_teleop.back,
        b'A': ptu2_teleop.left,       b'a': ptu2_teleop.left,
        b'D': ptu2_teleop.right,      b'd': ptu2_teleop.right,
        b' ': ptu2_teleop.reset}

    print("           yaw   ")
    print("                 ")
    print("            W    ")
    print("pitch   A       D")
    print("            S    ")
    print("                 ")
    print("reset:    space  ")
    print("                 ")
    print("speed +:    Q    ")
    print("speed -:    E    ")
    print("step +:     R    ")
    print("step -:     F    ")
    print("                 ")
    print("Ctrl+C to exit.  ")

    while True:
        inp = getch().encode('ASCII')
        method = key_map.get(inp)
        if method: method()