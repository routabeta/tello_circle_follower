import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

import sys
import tty
import termios
import select
import time

def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        else:
            return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_cli = self.create_client(TelloAction, '/tello_action')

        self.lateral_keys = {
            'i': (1, 0),
            'j': (0, 1),
            'k': (0, 0),
            'l': (0, -1),
            ',': (-1, 0),
        }
        self.alt_keys = {
            'w': (1, 0),
            's': (-1, 0),
            'a': (0, 1),
            'd': (0, -1),
        }
        self.cmd_keys = {
            't': 'takeoff',
            'g': 'land',
            '0': 'land',
            '1': 'land',
            '2': 'land',
            '3': 'land',
            '4': 'land',
            '5': 'land',
            '6': 'land',
            '7': 'land',
            '8': 'land',
            '9': 'land',
            'b': 'battery?'
        }

    def send_twist(self, lx=0, ly=0, lz=0, az=0):
        msg = Twist()
        msg.linear.x = float(lx)
        msg.linear.y = float(ly)
        msg.linear.z = float(lz)
        msg.angular.z = float(az)
        self.cmd_pub.publish(msg)

    def send_tello_command(self, cmd: str):
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for tello_action service...')
        req = TelloAction.Request()
        req.cmd = cmd
        future = self.cmd_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"Command '{cmd}' response: {future.result().rc}")
        else:
            self.get_logger().error(f"Failed to send command: {cmd}")

    def run(self):
        while rclpy.ok():
            key = get_key()

            if key in self.lateral_keys:
                lx, ly = self.lateral_keys[key]
                self.send_twist(lx=lx, ly=ly)
            elif key in self.alt_keys:
                lz, az = self.alt_keys[key]
                self.send_twist(lz=lz, az=az)
            elif key in self.cmd_keys:
                self.send_tello_command(self.cmd_keys[key])
            elif key == '\x03':  # Ctrl+C
                break
            
def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    node.get_logger().info("Control the drone using keys: i, j, k, l, , for lateral movement")
    node.get_logger().info("Control the drone using keys: w, s for up and down")
    node.get_logger().info("Control the drone using keys: a, d for yaw")
    node.get_logger().info("t = takeoff, g = land, b = battery?")

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
