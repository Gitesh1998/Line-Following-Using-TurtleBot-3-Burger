import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LineFollowerWithObstacleDetection(Node):
    def __init__(self):
        super().__init__('line_follower_with_obstacle_detection')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.process_frame)

        self.direction = "Go Straight"
        self.obstacle_detected = False

    def scan_callback(self, msg):
        self.obstacle_detected = any(distance < 0.3 for distance in msg.ranges if distance > 0.0)

    def process_frame(self):
        if self.obstacle_detected:
            self.get_logger().info('Obstacle detected within 30 cm! Stopping the bot.')
            self.stop_movement()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video ended or camera feed lost")
            return

        small_frame = cv2.resize(frame, (160, 120))  # Resize to 160x120

        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                height, width = small_frame.shape[:2]
                center_x = width // 2
                if cx < center_x - width * 0.1:
                    self.direction = "Turn Left"
                elif cx > center_x + width * 0.1:
                    self.direction = "Turn Right"
                else:
                    self.direction = "Go Straight"
            else:
                self.direction = "Line lost"
        else:
            self.direction = "Line not detected"

        self.execute_movement()

    def execute_movement(self):
        twist = Twist()
        if self.direction == "Go Straight":
            twist.linear.x = 0.02
            twist.angular.z = 0.0
        elif self.direction == "Turn Left":
            twist.linear.x = 0.0
            twist.angular.z = 0.05
        elif self.direction == "Turn Right":
            twist.linear.x = 0.0
            twist.angular.z = -0.05
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info(f"Direction: {self.direction}")

    def stop_movement(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Robot stopped. Velocities set to 0.")


    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    line_follower_with_obstacle_detection = LineFollowerWithObstacleDetection()

    try:
        rclpy.spin(line_follower_with_obstacle_detection)
    except KeyboardInterrupt:
        line_follower_with_obstacle_detection.get_logger().info("Ctrl+C detected! Stopping the bot.")
        line_follower_with_obstacle_detection.stop_movement()
    finally:
        line_follower_with_obstacle_detection.cleanup()


if __name__ == '__main__':
    main()
