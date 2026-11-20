# Note: Still missing import

# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import Image
# pyrefly: ignore [missing-import]
from std_msgs.msg import String
# pyrefly: ignore [missing-import]
from cv_bridge import CvBridge
# pyrefly: ignore [missing-import]
import cv2
import numpy as np
import json

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

        # Publisher untuk hasil deteksi — akan dikonsumsi oleh web monitoring via rosbridge
        self.detection_pub = self.create_publisher(
            String,
            '/detection_results',
            10
        )
        
        self.get_logger().info('Object Detection Node has been started.')
        self.get_logger().info('Publishing detection results to /detection_results')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # --- YOLO Inference ---
        # Uncomment blok di bawah setelah install ultralytics
        # results = self.model(cv_image)
        # detections = []
        # 
        # for r in results:
        #     boxes = r.boxes
        #     for box in boxes:
        #         # Bounding box coordinates
        #         x1, y1, x2, y2 = box.xyxy[0]
        #         x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        #         
        #         # Confidence dan class
        #         conf = float(box.conf[0])
        #         cls_id = int(box.cls[0])
        #         label = self.model.names[cls_id]
        #         
        #         detections.append({
        #             "label": label,
        #             "confidence": conf,
        #             "bbox": [x1, y1, x2, y2]
        #         })
        #         
        #         # Draw bounding box
        #         cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        #         cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1 - 10),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        # 
        # # Publish hasil deteksi sebagai JSON string ke /detection_results
        # if detections:
        #     det_msg = String()
        #     det_msg.data = json.dumps(detections)
        #     self.detection_pub.publish(det_msg)
        #     self.get_logger().debug(f'Published {len(detections)} detections')
        
        # --- Placeholder: publish dummy detection saat YOLO belum aktif ---
        # Hapus blok ini setelah YOLO model di-uncomment
        detections = []
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detection_pub.publish(det_msg)

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
