#!/usr/bin/env python3
from ros2_light_py.utilize import quat2Euler, Info, Euler
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8MultiArray

class MonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("monitor_node")
        self.imu_subscription = self.create_subscription(
            Imu,
            "imu_plugin/out",
            self.imu_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )
        self.button_subscription = self.create_subscription(
            Int8MultiArray,
            "joy_button",
            self.button_callback,
            10
        )

        self.info: Info = {
            "pose": [0.0, 0.0],
            "euler": Euler(0.0, 0.0, 0.0),
            "velocity": 0.0,
            "button": [0]
        }
        
    def imu_callback(self, msg: Imu) -> None:
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        self.info["euler"] = quat2Euler(x, y, z, w)
        # self.get_logger().info(f"{xyz[0]=:.2f} {xyz[1]=:.2f} {xyz[2]=:.2f}")

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.info["pose"] = [x, y]

        vX = msg.twist.twist.linear.x
        vY = msg.twist.twist.linear.y
        self.info["velocity"] = pow(pow(vX, 2) + pow(vY, 2), 0.5)
        # self.get_logger().info(f"{velocity=}")

    def button_callback(self, msg: Int8MultiArray) -> None:
        data = msg.data
        self.info["button"] = data