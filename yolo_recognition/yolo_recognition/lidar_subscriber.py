import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import PersonArray, Entities
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import Marker, MarkerArray
from .include.transform import GeometricTransformations

# Global variables
visual_data = PersonArray()
lidar_data = []

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.cb_group = ReentrantCallbackGroup()
        self.laser_sub_ = self.create_subscription(LaserScan, "/rplidar_controller/out", self.lidar_callback, 10, callback_group=self.cb_group)
        self.laser_pub_ = self.create_publisher(Entities, "/laser_data_array", 10)
        self.transform = GeometricTransformations(self)


    def lidar_callback(self, lidar_data_msg):
        global lidar_data, visual_data

        lidar_ranges = np.array(lidar_data_msg.ranges)
        lidar_angle_min = lidar_data_msg.angle_min
        lidar_angle_increment = lidar_data_msg.angle_increment

        lidar_angles = np.arange(lidar_angle_min, lidar_angle_min + len(lidar_ranges) * lidar_angle_increment, lidar_angle_increment)
        lidar_x = lidar_ranges * np.cos(lidar_angles)
        lidar_y = lidar_ranges * np.sin(lidar_angles)

        lidar_points = np.vstack((lidar_x, lidar_y)).T

        # variables for tracking the person 
        threshold, ids, classes, updated_lidar_data = 0.75, [], [], []

        for i, (person_x, person_y) in enumerate(lidar_data):
            distances = np.linalg.norm(lidar_points - np.array([person_x, person_y]), axis=1)
            points_within_circle = np.where(distances <= threshold)[0]

            if len(points_within_circle) == 0:
                self.get_logger().warn(f"Person {i} not found in LiDAR data")
                continue

            mean_lidar_x = np.mean(lidar_points[points_within_circle, 0])
            mean_lidar_y = np.mean(lidar_points[points_within_circle, 1])

            updated_lidar_data.append([mean_lidar_x, mean_lidar_y])
            ids.append(i)
            
        lidar_data = updated_lidar_data

        visual_points = np.array([[person.x, person.y] for person in visual_data.persons_array])

        if len(lidar_data) == 0:
            lidar_data = visual_points.tolist()
        else:
            threshold = 2.0
            new_lidar_data = lidar_data.copy()

            for visual_point in visual_points:
                distances = np.linalg.norm(np.array(lidar_data) - visual_point, axis=1)
                min_distance = np.min(distances)

                if min_distance > threshold:
                    new_lidar_data.append(visual_point.tolist())
                else:
                    index = np.argmin(distances)
                    new_lidar_data[index] = visual_point.tolist()

            lidar_data = new_lidar_data
            lidar_data = list(set(map(tuple, lidar_data)))

            # # transform the lidar data to the map frame
            # transformation = self.transform.get_transformation("map", "rplidar_link")
            # obs_arr = np.hstack((np.array(lidar_data[:,0])), np.array(lidar_data[:,1]), np.zero_like(lidar_data[:,0]), np.ones_like(lidar_data[:,0])).reshape(4,-1)

            # if transformation is not None:
            #     transformed_points = self.transform.transform_points(obs_arr, transformation)
            #     lidar_data = transformed_points.T

            entities = Entities()
            entities.count = len(lidar_data)
            entities.id = ids
            entities.x = np.array(lidar_data)[:, 0].tolist()
            entities.y = np.array(lidar_data)[:, 1].tolist()

            self.laser_pub_.publish(entities)

        

class VisualDataSubscriber(Node):
    def __init__(self):
        super().__init__("visual_data_subscriber")
        self.cb_group = ReentrantCallbackGroup()
        self.visual_sub_ = self.create_subscription(PersonArray, "/visual_dynamic_obs_array", self.visual_callback, 10, callback_group=self.cb_group)

    def visual_callback(self, visual_msg):
        global visual_data, lidar_data
        visual_data = visual_msg

        
def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()
    visual_data_subscriber = VisualDataSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(lidar_subscriber)
    executor.add_node(visual_data_subscriber)
    executor.spin()

if __name__ == "__main__":
    main()

