import rclpy
from rclpy.node import Node
from .include.marker_visualizer import MarkerVisualizer

class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")
        self.marker_visualizer = MarkerVisualizer(self, "/laser_data_array", "/lidar_markers", "rplidar_link")

def main(args=None):
    rclpy.init(args=args)
    visualizer = Visualizer()
    rclpy.spin(visualizer)
    rclpy.shutdown()