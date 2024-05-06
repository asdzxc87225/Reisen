import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        timer_period = 1/20  # seconds
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        # Capture image from your camera
        # For demonstration purposes, we'll create a simple image
        ret, image = self.cap.read()
        if image is None:
            print("Failed to load image")
            return
        # Convert the image to ROS format
        ros_image = self.cv_bridge.cv2_to_imgmsg(image, 'bgr8')
        # Set the frame id
        ros_image.header.frame_id = "camera_depth_frame"
        # Publish the image
        self.publisher_.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

