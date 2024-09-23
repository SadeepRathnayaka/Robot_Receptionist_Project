import rclpy
from rclpy.node import Node
import numpy as np
from custom_msgs.msg import LaserVisualData, PersonInfo, PersonArray, LaserVisualDataArray  # Replace with your custom message types
from visualization_msgs.msg import Marker, MarkerArray  # For visualizing in RViz
from std_msgs.msg import Header

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Kalman filter parameters
        self.Q = np.array([[0.1, 0], [0, 0.1]])  # Process noise covariance
        self.R = np.array([[0.05, 0], [0, 0.05]])  # Measurement noise covariance
        self.P = np.array([[1, 0], [0, 1]])  # State covariance

        # Subscribe to the sensor fusion topic (camera and lidar data)
        self.sensor_sub = self.create_subscription(
            LaserVisualDataArray,
            'laser_visual_data_array',
            self.sensor_callback,
            10
        )

        # Publisher for the fused sensor data
        self.fused_pub = self.create_publisher(PersonArray, 'fused_data', 10)

        # MarkerArray publisher for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'fused_markers', 10)

    def sensor_callback(self, msg):
        fused_data = PersonArray()
        fused_data.header = msg.header

        # Initialize MarkerArray
        marker_array = MarkerArray()

        for i, person in enumerate(msg.data_array):
            # Kalman filter prediction using visual data
            X_pred = np.array([person.visual_x, person.visual_y])

            # Kalman filter measurement using LiDAR data
            Z = np.array([person.laser_x, person.laser_y])

            # Kalman gain
            K = np.dot(self.P, np.linalg.inv(self.P + self.R))

            # State update
            X_fused = X_pred + np.dot(K, (Z - X_pred))

            # Update covariance matrix
            self.P = np.dot((np.eye(2) - K), self.P)

            # Create a fused person data
            fused_person = PersonInfo()
            fused_person.name = person.name
            fused_person.x = X_fused[0]
            fused_person.y = X_fused[1]
            

            # Append fused data to the array
            fused_data.persons_array.append(fused_person)

            # Create marker for the person
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'fused_people'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Set position to fused (x, y) and z as 0 (2D)
            marker.pose.position.x = X_fused[0]
            marker.pose.position.y = X_fused[1]
            marker.pose.position.z = 0.0

            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.001

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

            # Add marker to marker array
            marker_array.markers.append(marker)

        # Publish fused sensor data
        self.fused_pub.publish(fused_data)

        # Publish the marker array for visualization
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
