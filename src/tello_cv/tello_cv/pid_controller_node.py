import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        self.dist_sub = self.create_subscription(Vector3, "/circle_offset", self.dist_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.declare_parameter('target_radius_param', 60)
        self.targetRadius = float(self.get_parameter('target_radius_param').value)

        self.xd = 0 # x diff
        self.yd = 0 # y diff
        self.rd = self.targetRadius # distance diff, initialize to be correct
        self.pxd = 0 # previous x diff
        self.pyd = 0 # previous y diff
        self.prd = 0 # previous dist diff
        self.xderiv = 0
        self.yderiv = 0
        self.rderiv = 0
        self.kp = 0.0008
        self.ki = 0 # Unused
        self.kd = 0.001

    def dist_callback(self, msg):
        self.xd, self.yd = msg.x, msg.y # If positive then relative to ball location the drone must x: move right, y: move down
        self.rd = self.targetRadius - msg.z # If positive, circle is too small -> go forwards
        self.xderiv = self.xd - self.pxd # if positive, drone is moving left, so move right
        self.yderiv = self.yd - self.pyd # if positive, drone is moving up, so move down
        self.rderiv = self.rd - self.prd # if positive, circle is getting bigger, so move backwards

        # xd is negative when ball is left, but msg.linear.y > 0 moves left, so add - sign
        # xderiv is positive when ball is moving right (drone left), meaning drone must move right which is -, so add - sign
        
        # Note that x/y on image are y/z irl
        correctionx = self.kp *  12 *    self.rd +    self.kd * -12 * self.rderiv # Multiply kp/kd as its a 'weak signal'
        correctiony = self.kp * -1 *    self.xd +    self.kd * -1 * self.xderiv
        correctionz = self.kp * -3 *    self.yd +    self.kd * -3 * self.yderiv # Multply slightly to compensate for weaker control/smaller FOV along z axis

        self.get_logger().info("Go for (pos) or back (neg) by %f" % correctionx)
        self.get_logger().info("Go left (pos) or right (neg) by %f" % correctiony)
        self.get_logger().info("Go up (pos) or down (neg) by %f" % correctionz)

        msg = Twist()
        msg.linear.x = correctionx
        msg.linear.y = correctiony
        msg.linear.z = correctionz
        self.twist_pub.publish(msg)
        self.pxd, self.pyd, self.prd = self.xd, self.yd, self.rd

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return 0