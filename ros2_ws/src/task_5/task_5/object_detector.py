import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from vision_msgs.msg import BoundingBox2D

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Subscribe to video feed
        self.subscription = self.create_subscription(
            Image,
            '/video_data',
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Publisher for bounding boxes
        self.bbox_pub = self.create_publisher(BoundingBox2D, '/bbox', 10)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Create OpenCV window
        cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV frame
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        # Detect object (example: detect red objects)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Red color range (adjust these values for your object)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Print centroid and dimensions (pixels)
            centroid_x = x + w/2
            centroid_y = y + h/2
            self.get_logger().info(f"Centroid: ({centroid_x:.1f}, {centroid_y:.1f}), Size: {w}x{h}")
            
            # Publish BoundingBox2D message
            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = centroid_x
            bbox_msg.center.position.y = centroid_y
            bbox_msg.size_x = float(w)
            bbox_msg.size_y = float(h)
            self.bbox_pub.publish(bbox_msg)
            
            # Draw bounding box and centroid
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (int(centroid_x), int(centroid_y)), 5, (0, 0, 255), -1)
        
        # Display frame
        cv2.imshow('Object Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
