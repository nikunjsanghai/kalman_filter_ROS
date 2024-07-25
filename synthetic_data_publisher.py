import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
import time

class ComplexSyntheticDataPublisher(Node):
    def __init__(self):
        super().__init__('complex_synthetic_data_publisher')
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.gps_publisher = self.create_publisher(NavSatFix, 'gps/data', 10)
        self.baseline_publisher = self.create_publisher(NavSatFix, 'baseline/data', 10)
        
        # Adding a delay before starting the timers
        self.get_logger().info('Waiting for 10 seconds before starting...')
        time.sleep(10)  # Delay for 10 seconds
        self.get_logger().info('10 seconds have passed, starting timers...')
        
        self.imu_timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz
        self.gps_timer = self.create_timer(0.1, self.publish_gps_data)   # 10 Hz
        self.baseline_timer = self.create_timer(0.01, self.publish_baseline_data)  # 100 Hz
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Simulation parameters
        self.lat = 37.7749  # Starting latitude (example: San Francisco)
        self.lon = -122.4194  # Starting longitude
        self.alt = 10.0  # Starting altitude in meters
        self.speed = 30.0  # Starting speed in m/s (108 km/h)
        self.heading = 0.0  # Starting heading in radians
        self.acceleration = 0.0  # Starting acceleration

    def publish_imu_data(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.update_vehicle_state(current_time)
        
        # Generate IMU data with noise
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        # Simulate linear acceleration with noise
        imu_msg.linear_acceleration.x = float(self.acceleration + np.random.normal(0, 0.25))
        imu_msg.linear_acceleration.y = float(np.random.normal(0, 0.25))
        imu_msg.linear_acceleration.z = float(np.random.normal(0, 0.1))
        # Simulate angular velocity with noise
        imu_msg.angular_velocity.x = float(np.random.normal(0, 0.1))
        imu_msg.angular_velocity.y = float(np.random.normal(0, 0.1))
        imu_msg.angular_velocity.z = float(self.calculate_angular_velocity(current_time) + np.random.normal(0, 0.1))
        
        self.imu_publisher.publish(imu_msg)

    def publish_gps_data(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.update_vehicle_state(current_time)
        
        # Generate GPS data with noise
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'base_link'
        gps_msg.latitude = float(self.lat + np.random.normal(0, 5e-6))  # Small noise
        gps_msg.longitude = float(self.lon + np.random.normal(0, 5e-6))  # Small noise
        gps_msg.altitude = float(self.alt + np.random.normal(0, 0.000005))  # Small noise
        
        self.gps_publisher.publish(gps_msg)
    
    def publish_baseline_data(self):
        baseline_msg = NavSatFix()
        baseline_msg.header.stamp = self.get_clock().now().to_msg()
        baseline_msg.header.frame_id = 'base_link'
        baseline_msg.latitude = float(self.lat)
        baseline_msg.longitude = float(self.lon)
        baseline_msg.altitude = float(self.alt)  # Assuming altitude remains constant for baseline
        
        self.baseline_publisher.publish(baseline_msg)

    def update_vehicle_state(self, current_time):
        # Update vehicle state based on predefined maneuvers
        if current_time < 10:
            self.acceleration = 2.0  # Accelerate for the first 10 seconds
        elif current_time < 20:
            self.acceleration = 0.0  # Maintain speed
        elif current_time < 25:
            self.acceleration = -2.0  # Decelerate (emergency stop)
        elif current_time < 35:
            self.acceleration = 2.0  # Accelerate again
        elif current_time < 45:
            self.acceleration = 0.0  # Maintain speed
        elif current_time < 50:
            self.heading += np.deg2rad(45)  # Change lane
        elif current_time < 60:
            self.heading -= np.deg2rad(45)  # Change lane back
        else:
            self.acceleration = 0.0  # Maintain speed
        
        self.speed += self.acceleration * 0.1  # Update speed
        self.lat += (self.speed * 0.1 * np.cos(self.heading)) / 111139.0  # Update latitude
        self.lon += (self.speed * 0.1 * np.sin(self.heading)) / (111139.0 * np.cos(np.deg2rad(self.lat)))  # Update longitude

    def calculate_angular_velocity(self, current_time):
        # Calculate angular velocity based on heading changes
        if current_time < 10:
            return 0.0
        elif current_time < 20:
            return 0.0
        elif current_time < 25:
            return 0.0
        elif current_time < 35:
            return 0.0
        elif current_time < 45:
            return 0.0
        elif current_time < 50:
            return np.deg2rad(45) / 5.0  # Change lane over 5 seconds
        elif current_time < 60:
            return -np.deg2rad(45) / 5.0  # Change lane back over 5 seconds
        else:
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = ComplexSyntheticDataPublisher()
    
    # Create a timer to shutdown after 70 seconds
    def shutdown_callback():
        node.get_logger().info('70 seconds have passed, shutting down...')
        rclpy.shutdown()
    
    node.create_timer(70.0, shutdown_callback)  # 70 seconds timer
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
