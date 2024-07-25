### Kalman Filter in ROS2 Humble
Steps involved in implementation
- used synthetic_data_publisher.py to create synthetic IMU and GPS data with added guassian noise
- recorded the imu, gps and baseline data in ROS bag file
- built package ros2 package kalman_filter
- created launch file kalman_launch_file.py to run kalman_filter package and store results in a bag file
- finally plotted results for low and high gaussian noise data points using plot_kalman_filter.py

### Results 
- with low sensor noise
![Figure_2](https://github.com/user-attachments/assets/75d05896-6983-4f34-a22b-2d5910b247c8)
-with high sensor noise
![Figure_1](https://github.com/user-attachments/assets/eb3e5f39-fff2-463e-9f15-84958ad2bcee)

