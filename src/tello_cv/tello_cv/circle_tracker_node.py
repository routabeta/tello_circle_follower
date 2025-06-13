import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
from tello_cv.circle_finder import find_circles, tracker_init

class CircleTracker(Node):
    def __init__(self):
        super().__init__("circle_tracker_node")
        self.img_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.img_pub = self.create_publisher(Image, "/image_circled", 10)
        self.dist_pub = self.create_publisher(Vector3, "/circle_offset", 10)

        self.bridge = CvBridge()
        self.tracker = None

        self.declare_parameter('scale_param', 1)
        self.declare_parameter('max_tracker_err_param', 20)
        self.declare_parameter('target_radius_param', 60)

        self.scale = self.get_parameter('scale_param').value
        self.max_tracker_err = self.get_parameter('max_tracker_err_param').value
        self.targetRadius = float(self.get_parameter('target_radius_param').value) # How far we want the drone to sit

        self.maxRadius = int(120 * self.scale)
        self.minRadius = int(2 * self.scale)
    
    # TRACKING LOCATION OF CIRCLE
    # node should make calls to find_circle to update circle location when possible
    # node should take location/bounding for located circle and pass it to tracker in case the bounds drop:
        # If bounds do not drop, integrate some sort of filter?
        # If bounds DO drop, use tracker

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        success = False
        tracker_x, tracker_y = None, None
        # Try to update tracker if initialized
        if self.tracker is not None: 
            success, bbox = self.tracker.update(frame)
            if success:
                tracker_x, tracker_y, w, h = [int(v) for v in bbox]
                r = w//2
                cv2.circle(frame, (tracker_x + r, tracker_y + r), 2, (255, 255, 255), 2)
                cv2.rectangle(frame, (tracker_x, tracker_y), (tracker_x + w, tracker_y + h), (255, 255, 255), 2)
                cv2.line(frame, (tracker_x + r, tracker_y + r), (frame.shape[1]//2, frame.shape[0]//2), (255, 255, 255), 2)
        else:
            self.get_logger().warn("Tracker is not initialized")

        # Identify circles in the image and draw/update as applicable
        circles = find_circles(frame, self.scale, maxRadius=self.maxRadius, minRadius=self.minRadius)
        if circles is not None and len(circles) == 1:
            x, y, r = map(lambda n: int(n / self.scale), circles[0])
            cv2.circle(frame, (x, y), 2, (0, 255, 255), 2)
            cv2.circle(frame, (x, y), r, (255, 0, 0), 2)
            cv2.line(frame, (x, y), (frame.shape[1]//2, frame.shape[0]//2), (255, 0, 255), 2)

            # Only reinit tracker if it's too far from most recent circle update
            if self.tracker is None or not success or (tracker_x - x)**2 + (tracker_y - y)**2 > self.max_tracker_err**2:
                self.tracker = tracker_init(frame, x, y, r)

            # Publish offset
            diff_msg = Vector3()
            diff_msg.x = float(x - frame.shape[1]//2) # negative means its to the left
            diff_msg.y = float(y - frame.shape[0]//2) # negative means its above
            diff_msg.z = float(r) # send radius
            self.dist_pub.publish(diff_msg)
        elif success: # Use tracker data (if available) if circles are not
            diff_msg = Vector3()
            diff_msg.x = float(tracker_x - frame.shape[1]//2) # negative means its to the left
            diff_msg.y = float(tracker_y - frame.shape[0]//2) # negative means its above
            diff_msg.z = float(r) # send radius
            self.dist_pub.publish(diff_msg)
        else: # No cirlces or tracker, stop
            diff_msg = Vector3()
            diff_msg.x = 0.0
            diff_msg.y = 0.0
            diff_msg.z = float(self.targetRadius) # send radius
            self.dist_pub.publish(diff_msg)



        if circles is not None and len(circles) > 1:
            self.get_logger().warn("Multiple circles detected")


        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header  # Preserve timestamp/frame info
        self.img_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CircleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()