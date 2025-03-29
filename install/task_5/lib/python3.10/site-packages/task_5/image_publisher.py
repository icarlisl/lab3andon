# Import necessary libraries
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Create a publisher to send images over ROS 2
        self.publisher_ = self.create_publisher(Image, '/video_data', 10)

        # Initialize OpenCV-to-ROS2 bridge
        self.bridge = CvBridge()

        # Load the video file
        video_path = os.path.join(
            get_package_share_directory('task_5'),  # Find package directory
            'resource',                             # Subfolder
            'lab3_video.avi'                        # Video filename
        )

        # Open the video file
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file!")
            raise RuntimeError("Video file not found or corrupted")

        # Set up a timer to publish frames at video FPS
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 30  # Default to 30 FPS if invalid
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info("Publishing video frames...")

    def timer_callback(self):
        # Read a frame from the video
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().info("End of video reached")
            self.timer.cancel()  # Stop the timer
            self.cap.release()   # Release the video file
            return

        try:
            # Convert OpenCV frame to ROS 2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # Publish the image
            self.publisher_.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
