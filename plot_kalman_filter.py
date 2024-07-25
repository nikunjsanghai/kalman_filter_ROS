import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

class PlotKalmanFilter(Node):
    def __init__(self):
        super().__init__('plot_kalman_filter')
        self.baseline_data = []
        self.estimate_data = []
        self.origin_lat = None
        self.origin_lon = None

    def read_bag_file(self, bag_path):
        print(f"Reading bag file from: {bag_path}")
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(storage_options, converter_options)
            print("Bag file opened successfully.")
        except Exception as e:
            print(f"Failed to open bag file: {e}")
            return

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        print(f"Topics in the bag file: {type_map}")

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            if topic == '/baseline/data':
                if self.origin_lat is None and self.origin_lon is None:
                    self.origin_lat = msg.latitude
                    self.origin_lon = msg.longitude
                x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
                self.baseline_data.append((x, y))
            elif topic == '/kalman_filter/estimate':
                x, y = self.latlon_to_xy(msg.data[0], msg.data[1])
                self.estimate_data.append((x, y))

    def latlon_to_xy(self, lat, lon):
        # Approximation assuming a flat Earth
        R = 6371000  # radius of Earth in meters
        lat_rad = np.radians(lat)
        lon_rad = np.radians(lon)
        origin_lat_rad = np.radians(self.origin_lat)
        origin_lon_rad = np.radians(self.origin_lon)

        x = R * (lon_rad - origin_lon_rad) * np.cos((lat_rad + origin_lat_rad) / 2)
        y = R * (lat_rad - origin_lat_rad)
        return x, y

    def plot_data(self):
        if not self.baseline_data or not self.estimate_data:
            print("No data to plot.")
            return
        baseline_data = np.array(self.baseline_data)
        estimate_data = np.array(self.estimate_data)
        plt.figure()
        plt.plot(baseline_data[:, 0], baseline_data[:, 1], 'b', label='Baseline')
        plt.plot(estimate_data[:, 0], estimate_data[:, 1], 'r', label='Kalman Filter')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.legend()
        plt.title('Baseline vs Kalman Filter in Meters')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PlotKalmanFilter()
    bag_path = '/home/nikunj/ros2_ws/kalman_filter_results'  # Update this path as needed
    node.read_bag_file(bag_path)
    node.plot_data()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

