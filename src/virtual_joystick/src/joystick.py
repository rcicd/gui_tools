#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gui import GUI
from sensor_msgs.msg import Joy, Range
from std_msgs.msg import Header
import os


class VirtualJoystick(Node):
    def __init__(self):
        super(VirtualJoystick, self).__init__(node_name="virtual_joystick_node")
        self.declare_parameter("vehicle", "")
        self.declare_parameter("assets_dir", "assets")
        self.declare_parameter("movement_states", ["a:autonomous"])
        self.declare_parameter("toggle_states", ["o:Toggle OrbSlam node:5"])
        self.vehicle = self.get_parameter("vehicle").get_parameter_value().string_value
        self.assets_dir = self.get_parameter("assets_dir").get_parameter_value().string_value
        self._movement_states = self.get_parameter("movement_states").value
        self._toggle_states = self.get_parameter("toggle_states").value
        self.publisher = self.create_publisher(Joy, f"/{self.vehicle}/virtual_joystick", 1)
        self.get_logger().info(f"{self._toggle_states}")
        self.movement_states = {element[0]: element[1] for element in [state.split(":") for state in self._movement_states]}
        self.toggle_states = {element[0]: (element[1], int(element[2])) for element in [state.split(":") for state in self._toggle_states]}
        self.gui = GUI(
            callback=lambda buttons, scale: self.callback(buttons, scale),
            assets_dir=self.assets_dir,
            toggle_states=self.toggle_states,
            movement_states=self.movement_states,
            logger=self.get_logger()
        )
        rclpy.shutdown()

    def callback(self, buttons, scale):
        msg = Joy()
        msg.header = Header(stamp=rclpy.time.Time().to_msg())
        msg.buttons = buttons
        msg.axes = [scale]
        self.publisher.publish(msg)

    def on_shutdown(self):
        self.gui.on_shutdown()


if __name__ == "__main__":
    os.system('xset r off')
    rclpy.init()
    node = VirtualJoystick()
    os.system('xset r on')
