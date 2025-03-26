#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            self.camera_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Camera subscriber node started')
        
        # Create a resizable window
        cv2.namedWindow('Camera Feed Processing', cv2.WINDOW_NORMAL)
        
    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get original dimensions
            height, width, _ = cv_image.shape
            
            # Resize to a more manageable size if too large
            max_width = 640 #1280
            if width > max_width:
                scale_factor = max_width / width
                new_width = int(width * scale_factor)
                new_height = int(height * scale_factor)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
                height, width, _ = cv_image.shape
            
            # Add text overlay with resolution
            text = f"Resolution: {width}x{height}"
            cv2.putText(cv_image, text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Example processing: Edge detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            
            # Stack the original and processed images side by side
            processed_display = np.hstack((cv_image, edges_colored))
            
            # Display the result in a resizable window
            cv2.imshow("Camera Feed Processing", processed_display)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    
    # Clean up on shutdown
    cv2.destroyAllWindows()
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
