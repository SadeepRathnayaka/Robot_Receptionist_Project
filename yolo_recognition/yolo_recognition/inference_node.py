from ultralytics import YOLO
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from custom_msgs.msg import PersonInfo, PersonArray, Entities
from visualization_msgs.msg import Marker, MarkerArray
import math
import torch


bridge = CvBridge()

def distance_from_camera(height):
    return (310 * 3) / height

def angle_from_camera(hor_dis):
    return (hor_dis * 45.07) / 557

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/home/sadeep/fyp_ws/src/yolo_recognition/yolo_recognition/best.pt')

        self.sub_ = self.create_subscription(Image, '/zed2_left_camera/image_raw', self.camera_callback, 10)
        self.img_pub_ = self.create_publisher(Image, '/inference_result', 1)
        self.array_pub_ = self.create_publisher(Entities, '/visual_dynamic_obs_array', 1)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/visual_data_markers', 1)

        self.lidar_to_cam_opt = np.array([
            [0.000, 0.000, 1.000, -0.170],
            [-1.000, 0.000, 0.000, 0.060],
            [0.000, -1.000, 0.000, 1.099],
            [0.000, 0.000, 0.000, 1.000]
        ])

        # A dictionary to keep track of active markers by ID
        self.active_markers = {}

    def camera_callback(self, msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)

        # # Convert bounding boxes, class indices, and class float mappings to numpy arrays
        # xyxy_boxes = results[0].boxes.xyxy.cpu().numpy()  
        # class_indices = results[0].boxes.cls.cpu().numpy()  

        # class_floats = {0: 1.2, 1: 1.4, 2: 1.6, 3: 1.8}

        # # Create a numpy array of float values corresponding to class indices
        # float_values = np.vectorize(class_floats.get)(class_indices)

        # # Stack bounding boxes, class indices, and float values together
        # new_data = np.column_stack((xyxy_boxes, class_indices, float_values))

        # Mid-point of the image
        mid_point_x = int(img.shape[1] / 2)
        mid_point_y = int(img.shape[0] / 2)

        # Create a marker array for dynamic obstalces to publish
        # marker_array = MarkerArray()

        # detected_ids = set()  # Track detected markers in this frame

        # count = 0  # Use this to assign unique IDs

        classes, arr_x, arr_y = [] , [] , []

        for r in (results):
            boxes = r.boxes
            
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                conf = box.conf.item()
                if (self.model.names[int(c)] == "normal-adult"):

                    x_min = int(b[0])  # Top-left x-coordinate
                    y_min = int(b[1])  # Top-left y-coordinate
                    x_max = int(b[2])  # Bottom-right x-coordinate
                    y_max = int(b[3])  # Bottom-right y-coordinate

                    # Draw rectangle
                    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 255, 0), thickness=2)

                    # Height calculations
                    height = y_max - y_min
                    distance = distance_from_camera(height)

                    # Angle calculations, distance corrections
                    u = int((x_min + x_max) / 2)  # Center x-coordinate of BB 
                    horizontal_pixel = u - mid_point_x
                    angle = angle_from_camera(horizontal_pixel)
                    distance = distance / np.cos(np.radians(angle))

                    cam_x = distance * math.sin(math.radians(angle))
                    cam_y = 0.0
                    cam_z = distance * math.cos(math.radians(angle))

                    # multiply by the transformation matrix
                    lidar_x, lidar_y, lidar_z, _ = np.dot(self.lidar_to_cam_opt, [cam_x, cam_y, cam_z, 1])

                    # Create marker for visualization in RViz
                    # marker = Marker()
                    # marker.header.frame_id = "rplidar_link"  
                    # marker.header.stamp = self.get_clock().now().to_msg()
                    # marker.type = Marker.CYLINDER
                    # marker.id = count  
                    # marker.action = Marker.ADD

                    # Set marker position using distance and angle
                    # marker.pose.position.x = lidar_x
                    # marker.pose.position.y = lidar_y
                    # marker.pose.position.z = 0.0

                    # marker.scale.x = 0.25  
                    # marker.scale.y = 0.25
                    # marker.scale.z = 0.001

                    # marker.color.a = 1.0  # Opacity (1.0 is fully visible)
                    # marker.color.r = 0.0
                    # marker.color.g = 1.0
                    # marker.color.b = 0.0

                    # Add marker to the array
                    # marker_array.markers.append(marker)

                    # Draw label
                    label = f'D: {distance:.2f}m A: {angle:.2f} deg'
                    cv2.putText(img, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

                    classes.append(self.model.names[int(c)])
                    arr_x.append(lidar_x)
                    arr_y.append(lidar_y)

                    # detected_ids.add(count)  # Add this marker to detected IDs
                    # self.active_markers[count] = marker  # Update active markers

                    # Create custom message
                    # person_info = PersonInfo()
                    # person_info.name = "person_" + str(count) 
                    # person_info.x = lidar_x
                    # person_info.y = lidar_y
                    # dynamic_obs_array.persons_array.append(person_info)
                    # count += 1

        entities = Entities()
        entities.count = len(arr_x)
        entities.classes = classes
        entities.x = arr_x
        entities.y = arr_y

        # handle markers that are no longer detected
        # for marker_id in list(self.active_markers.keys()):
        #     if marker_id not in detected_ids:
        #         # Create a delete marker
        #         delete_marker = Marker()
        #         delete_marker.id = marker_id
        #         delete_marker.action = Marker.DELETE
        #         marker_array.markers.append(delete_marker)
        #         # Remove from active markers
        #         del self.active_markers[marker_id]

        # self.marker_pub_.publish(marker_array)
        self.array_pub_.publish(entities)

        img_msg = bridge.cv2_to_imgmsg(img)
        self.img_pub_.publish(img_msg)
        
       
def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()

