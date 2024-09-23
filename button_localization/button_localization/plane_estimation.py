import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import time

save_path = "/home/sadeep/fyp_ws/src/button_localization/localization/points.npy"

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor_node')

        self.declare_parameter("start_angle", 0)
        self.declare_parameter("end_angle", 45)

        self.start_angle = self.get_parameter("start_angle").value
        self.end_angle = self.get_parameter("end_angle").value
        
        self.subscription = self.create_subscription(LaserScan, '/rplidar_controller/out', self.lidar_callback, 10)
        self.lidar_readings = []

    def lidar_callback(self, msg):
        
        angle_min = msg.angle_min  # Minimum angle of the scan (in radians)
        angle_max = msg.angle_max  # Maximum angle of the scan (in radians)
        angle_increment = msg.angle_increment  # Angle increment per scan (in radians)
        
        start_angle = self.start_angle * (3.14159 / 180.0)   # Convert to radians
        end_angle = self.end_angle * (3.14159 / 180.0)       # Convert to radians
        angle_range = np.abs(end_angle - start_angle)        # Angle range in radians

        # lidar readings starting from the max angle and anticlockwise
        start_angle_remap = angle_max - end_angle
        end_angle_remap = start_angle_remap + angle_range

        start_index = int(start_angle_remap / angle_increment)
        end_index = int(end_angle_remap / angle_increment)

        filtered_ranges = np.array(msg.ranges[start_index:end_index+1])

        indices = np.arange(start_index, end_index + 1)
        filtered_angles = indices * angle_increment  # Angles in radians
        filtered_angles =  np.abs(angle_max) - filtered_angles    # mapping the angles from - to +

        filtered_angles_degrees = np.degrees(filtered_angles)

        output = np.column_stack((filtered_ranges, filtered_angles_degrees))
        self.lidar_readings.append(output)
        self.get_logger().info("Saving lidar readings...")
        
def main(args=None):

    rclpy.init(args=args)
    processor = LidarProcessor()

    start_time = time.time()
    duration = 5  # Duration to run in seconds

    while rclpy.ok():
        rclpy.spin_once(processor, timeout_sec=0.1)
        current_time = time.time()

        if current_time - start_time > duration:
            break
    
    arr = processor.lidar_readings
    np.save(save_path, arr)
    time.sleep(1)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
