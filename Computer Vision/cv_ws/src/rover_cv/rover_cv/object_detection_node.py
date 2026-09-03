import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Uncomment and install ultralytics for YOLO support
# from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Initialize YOLO model (placeholder)
        # self.model = YOLO('yolov8n.pt') 
        self.get_logger().info('YOLO Model placeholder initialized.')

        # Create a subscriber to the raw image topic from the rover
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Object Detection Node has been started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # --- YOLO Inference Placeholder ---
        # results = self.model(cv_image)
        # 
        # for r in results:
        #     boxes = r.boxes
        #     for box in boxes:
        #         # Bounding box coordinates
        #         x1, y1, x2, y2 = box.xyxy[0]
        #         x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        #         
        #         # Draw bounding box
        #         cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        #         
        #         # Draw label (Placeholder)
        #         cv2.putText(cv_image, "Object", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Display the image for debugging (can be disabled in production)
        cv2.imshow("Rover Camera Feed - YOLO Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
