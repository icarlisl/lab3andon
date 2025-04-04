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
        
        self.video_subscription = self.create_subscription(Image,'/video_data',self.image_callback,10)
        self.bounds_publisher = self.create_publisher(BoundingBox2D, '/bbox', 10)
        
        self.bridge = CvBridge() # opencv things
        cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
        
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # converts ros image to opencv

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_min = np.array([0, 120, 70]) # hue, saturation, value
        red_max = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, red_min, red_max)
      
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            detected_object = max(contours, key=cv2.contourArea) # maximum not techincally the correct object 
           
            x, y, w, h = cv2.boundingRect(detected_object) # makes bounds
            
            centroid_x = x + w / 2
            centroid_y = y + h / 2
            
            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = centroid_x
            bbox_msg.center.position.y = centroid_y
            bbox_msg.size_x = float(w) 
            bbox_msg.size_y = float(h)  
            self.bounds_publisher.publish(bbox_msg)
            
            # makes bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (int(centroid_x), int(centroid_y)), 5, (0, 0, 255), -1)
            
            self.get_logger().info(f"Centroid: ({centroid_x:.1f}, {centroid_y:.1f}), Size: {w}x{h} pixels")
        
        # video feed with bounding box
        cv2.imshow('Object Detection', frame)
        cv2.waitKey(1)  

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    
    rclpy.spin(object_detector)
    
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
