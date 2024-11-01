from pyjoystick import ThreadEventManager
from pyjoystick.sdl2 import Key, Joystick, run_event_loop
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray

class JoyStickNode(Node):
    def __init__(self) -> None:
        super().__init__("joystick_node")
        timer_period = 1e-2
        self.twist_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.button_publisher = self.create_publisher(Int8MultiArray, "joy_button", 10)
        self.create_timer(timer_period, self.twist_timer)
        self.create_timer(timer_period, self.button_timer)

        self.speed = 1.0
        self.turn = 4.0
        self.left_stick_x = 0.0
        self.left_stick_y = 0.0
        self.right_stick_x = 0.0
        self.right_stick_y = 0.0
        self.cross = 0
        self.thread = ThreadEventManager(event_loop=run_event_loop,
                                         add_joystick=self.add_joystick,
                                         remove_joystick=self.remove_joystick,
                                         handle_key_event=self.handle_key_event)
        self.thread.start()

    def twist_timer(self):
        msg = Twist()
        msg.linear.x  = self.speed * self.left_stick_y
        msg.linear.y  = 0.0
        msg.linear.z  = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.turn * self.right_stick_x
        self.twist_publisher.publish(msg)
        # self.get_logger().info(f"{self.left_stick_x=}\n{self.left_stick_y=}\n{self.right_stick_x=}\n{self.right_stick_y=}")

    def button_timer(self):
        msg = Int8MultiArray()
        msg.data = [self.cross]
        self.button_publisher.publish(msg)
    
    def handle_key_event(self, key: Key) -> None:
        _num  = key.number 
        _type = key.keytype
        value = key.value
        if _type == key.AXIS:
            match _num:
                case 0:
                    self.left_stick_x = -value
                case 1:
                    self.left_stick_y = -value
                case 3:
                    self.right_stick_x = -value
                case 4:
                    self.right_stick_y = -value
            return
        
        if _type == key.BUTTON:
            match _num:
                case 0:
                    self.cross = value


    def add_joystick(self, joy: Joystick) -> None:
        self.get_logger().info(f"Added {joy}")

    def remove_joystick(self, joy: Joystick) -> None:
        self.get_logger().info(f"Removed {joy}")