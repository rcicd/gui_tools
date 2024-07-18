#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gui import GUI
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


class VirtualJoystick(Node):
    def __init__(self):
        super(VirtualJoystick, self).__init__(node_name="virtual_joystick_node")
        self.declare_parameter("vehicle", "")
        self.declare_parameter("assets_dir", "assets")
        self.vehicle = self.get_parameter("vehicle").get_parameter_value().string_value
        self.assets_dir = self.get_parameter("assets_dir").get_parameter_value().string_value
        self.publisher = self.create_publisher(Joy, f"/{self.vehicle}/virtual_joystick", 1)
        self.gui = GUI(callback=lambda buttons, scale: self.callback(buttons, scale), assets_dir=self.assets_dir)

    def callback(self, buttons, scale):
        msg = Joy()
        msg.header = Header(stamp=rclpy.time.Time().to_msg())
        msg.buttons = buttons
        msg.axes = [scale]
        self.publisher.publish(msg)


    def on_shutdown(self):
        self.gui.on_shutdown()


if __name__ == "__main__":
    rclpy.init()
    node = VirtualJoystick()
    rclpy.spin(node)
    rclpy.shutdown()
