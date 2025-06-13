import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_cli = self.create_client(TelloAction, '/tello_action')

        self.lateral_keys = {
            'i': (1, 0),
            'j': (0, -1),
            'k': (0, 0),
            'l': (0, 1),
            ',': (-1, 0),
        }
        self.alt_keys = {
            'w': (1, 0),
            's': (-1, 0),
            'a': (0, -1),
            'd': (0, 1),
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


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    node.get_logger().info("Control the drone using keys: i, j, k, l, , for lateral movement")
    node.get_logger().info("Control the drone using keys: w, s for up and down")
    node.get_logger().info("Control the drone using keys: a, d for yaw")
    node.get_logger().info("t = takeoff, g = land, b = battery?")
    try:
        while rclpy.ok():
            key = input("Key: ").strip()
            if key in node.lateral_keys:
                lx, ly = node.lateral_keys[key]
                node.send_twist(lx=lx, ly=ly)
            elif key in node.alt_keys:
                lz, az = node.alt_keys[key]
                node.send_twist(lz=lz, az=az)
            elif key == 't':
                node.send_tello_command("takeoff")
            elif key == 'g':
                node.send_tello_command("land")
            elif key == 'b':
                node.send_tello_command("battery?")
            elif key == 'q':
                print("Quitting.")
                break
            else:
                print("Unknown key")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
