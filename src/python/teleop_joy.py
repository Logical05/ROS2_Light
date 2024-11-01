#!/usr/bin/env python3
import sys
from os import getcwd
package_name = "ros2_light"
sys.path.append(getcwd() + f"/src/{package_name}/.venv/lib/python3.10/site-packages")

import rclpy
from ros2_light_py.joystick_node import JoyStickNode

def main(args=None):
    rclpy.init(args=args)
    node = JoyStickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()