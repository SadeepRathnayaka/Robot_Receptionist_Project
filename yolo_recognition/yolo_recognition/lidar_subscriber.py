import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import PersonInfo, PersonArray
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import Marker, MarkerArray

# Global variable
visual_data = PersonArray()


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.cb_group = ReentrantCallbackGroup()
        self.laser_sub_ = self.create_subscription(LaserScan, "/rplidar_controller/out", self.lidar_callback, 10, callback_group=self.cb_group)
        self.marker_pub_ = self.create_publisher(MarkerArray, "/lidar_markers", 10)
        self.laser_pub_ = self.create_publisher(PersonArray, "/laser_data_array", 10)

        # A dictionary to keep track of active markers by ID
        self.active_markers = {}
        self.visual_data_len = 0
        

    def lidar_callback(self, lidar_data):

        lidar_ranges = np.array(lidar_data.ranges)
        lidar_angle_min = lidar_data.angle_min
        lidar_angle_increment = lidar_data.angle_increment

        # Calculate lidar points in Cartesian coordinates
        lidar_angles = np.arange(lidar_angle_min, lidar_angle_min + len(lidar_ranges) * lidar_angle_increment, lidar_angle_increment)
        lidar_x = lidar_ranges * np.cos(lidar_angles)
        lidar_y = lidar_ranges * np.sin(lidar_angles)

        # LiDAR points (x, y) in the 2D plane
        lidar_points = np.vstack((lidar_x, lidar_y)).T

        marker_array = MarkerArray()
        detected_ids = set()  # Track detected markers in this frame
        delete_markers = MarkerArray() # For deleting unused markers
        
        laser_data_array = PersonArray()
        laser_data_array.header.frame_id = "rplidar_link"
        laser_data_array.header.stamp = self.get_clock().now().to_msg()

        global visual_data
        visual_data_len_ = len(visual_data.persons_array)

        if (visual_data_len_ == 0):
            return

        if (visual_data_len_ != self.visual_data_len):
            self.visual_data_len = visual_data_len_
            cartesian_coords = np.array([[person.x, person.y] for person in visual_data.persons_array])

            # Radius threshold for circle (in meters)
            radius_threshold = 0.5

            for i, (person_x, person_y) in enumerate(cartesian_coords):
                distances = np.linalg.norm(lidar_points - np.array([person_x, person_y]), axis=1)
                points_within_circle = np.where(distances <= radius_threshold)[0]

                if len(points_within_circle) == 0:
                    continue

                mean_lidar_x = np.mean(lidar_points[points_within_circle, 0])
                mean_lidar_y = np.mean(lidar_points[points_within_circle, 1])

                # Create the marker for RViz visualization
                marker = Marker()
                marker.header.frame_id = "rplidar_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "persons"
                marker.id = i  
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD

                marker.pose.position.x = mean_lidar_x
                marker.pose.position.y = mean_lidar_y
                marker.pose.position.z = 0.0

                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.001

                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

                detected_ids.add(i)  
                self.active_markers[i] = marker 

                # Create custom message
                laser_data = PersonInfo()
                laser_data.name = "person_" + str(i)
                laser_data.x = mean_lidar_x
                laser_data.y = mean_lidar_y

                laser_data_array.persons_array.append(laser_data)

            # Remove unused markers by setting their action to DELETE
            for marker_id in list(self.active_markers.keys()):
                if marker_id not in detected_ids:
                    marker = self.active_markers.pop(marker_id)
                    marker.action = Marker.DELETE
                    delete_markers.markers.append(marker)

            # Publish the updated and deleted markers
            if len(delete_markers.markers) > 0:
                self.marker_pub_.publish(delete_markers)

        else:
            threshold = 0.75
            marker_array = MarkerArray() 
            laser_data_array = PersonArray()  
            laser_data_array.header.frame_id = "rplidar_link"
            laser_data_array.header.stamp = self.get_clock().now().to_msg()

            for marker_id, marker in self.active_markers.items(): 
                person_x = marker.pose.position.x
                person_y = marker.pose.position.y

                distances = np.linalg.norm(lidar_points - np.array([person_x, person_y]), axis=1)
                points_within_circle = np.where(distances <= threshold)[0]

                if len(points_within_circle) == 0:
                    continue  

                mean_lidar_x = np.mean(lidar_points[points_within_circle, 0])
                mean_lidar_y = np.mean(lidar_points[points_within_circle, 1])

                marker.pose.position.x = mean_lidar_x
                marker.pose.position.y = mean_lidar_y

                # Create custom message for LaserVisualData
                laser_data = PersonInfo()
                laser_data.name = "person_" + str(marker_id)
                laser_data.x = mean_lidar_x
                laser_data.y = mean_lidar_y

                laser_data_array.persons_array.append(laser_data)
                marker_array.markers.append(marker)  

        self.marker_pub_.publish(marker_array)
        self.laser_pub_.publish(laser_data_array)
           

class VisualDataSubscriber(Node):
    def __init__(self):
        super().__init__("visual_data_subscriber")
        self.cb_group = ReentrantCallbackGroup()
        self.visual_sub_ = self.create_subscription(PersonArray, "/visual_dynamic_obs_array", self.visual_callback, 10, callback_group=self.cb_group)

        # A dictionary to keep track of active markers by ID
        self.active_markers = {}

    def visual_callback(self, msg):
        global visual_data
        visual_data = msg
        
        
def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()
    visual_data_subscriber = VisualDataSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(lidar_subscriber)
    executor.add_node(visual_data_subscriber)
    executor.spin()
