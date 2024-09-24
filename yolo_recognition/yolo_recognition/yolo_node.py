from ultralytics import YOLO
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from custom_msgs.msg import PersonInfo, PersonArray
from visualization_msgs.msg import Marker, MarkerArray
import math
import torch

bridge = CvBridge()

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/home/sadeep/fyp_ws/src/yolo_recognition/yolo_recognition/best.pt')

        self.sub_ = self.create_subscription(Image, '/zed2_left_camera/image_raw', self.camera_callback, 10)
        self.img_pub_ = self.create_publisher(Image, '/inference_result', 1)

    def camera_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)

        # Convert bounding boxes, class indices, and class float mappings to numpy arrays
        xyxy_boxes = results[0].boxes.xyxy.cpu().numpy()  # Bounding box coordinates (x1, y1, x2, y2)
        class_indices = results[0].boxes.cls.cpu().numpy()  # Class indices

        # Define the float values for each class
        class_floats = {0: 1.2, 1: 1.4, 2: 1.6, 3: 1.8}

        # Create a numpy array of float values corresponding to class indices
        float_values = np.vectorize(class_floats.get)(class_indices)

        # Stack bounding boxes, class indices, and float values together
        new_data = np.column_stack((xyxy_boxes, class_indices, float_values))

        for r in (results):
            boxes = r.boxes
            
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                conf = box.conf.item()
                
                x_min = int(b[0])  # Top-left x-coordinate
                y_min = int(b[1])  # Top-left y-coordinate
                x_max = int(b[2])  # Bottom-right x-coordinate
                y_max = int(b[3])  # Bottom-right y-coordinate

                # Draw rectangle
                cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 255, 0), thickness=2)
                bb_class = self.model.names[int(c)]

                # Draw label
                label = f'class: {bb_class} '
                cv2.putText(img, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

        img_msg = bridge.cv2_to_imgmsg(img)
        self.img_pub_.publish(img_msg)
       
def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()

