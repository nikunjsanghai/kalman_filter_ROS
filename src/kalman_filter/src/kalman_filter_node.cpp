#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Dense>

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kalman_filter_node")
    {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&KalmanFilterNode::imu_callback, this, std::placeholders::_1));
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/data", 10, std::bind(&KalmanFilterNode::gps_callback, this, std::placeholders::_1));
        estimate_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/kalman_filter/estimate", 10);
        baseline_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/baseline/data", 10, std::bind(&KalmanFilterNode::baseline_callback, this, std::placeholders::_1));
        initialize_kalman_filter();
    }

private:
    void initialize_kalman_filter()
    {
        // Initialize Kalman filter matrices
        x_ = Eigen::VectorXd(4);  // State vector [pos_x, pos_y, vel_x, vel_y]
        P_ = Eigen::MatrixXd::Identity(4, 4);  // State covariance matrix
        F_ = Eigen::MatrixXd::Identity(4, 4);  // State transition matrix
        Q_ = Eigen::MatrixXd::Zero(4, 4);  // Process noise covariance matrix
        H_ = Eigen::MatrixXd::Zero(2, 4);  // Measurement matrix
        R_ = Eigen::MatrixXd::Identity(2, 2);  // Measurement noise covariance matrix

        // Measurement matrix
        H_(0, 0) = 1;
        H_(1, 1) = 1;

        // Process noise (accelerations)
        double sigma_ax = 0.1;  // Process noise standard deviation for acceleration in x (m/s^2)
        double sigma_ay = 0.1;  // Process noise standard deviation for acceleration in y (m/s^2)
        Q_(0, 0) = pow(0.5 * sigma_ax, 2);
        Q_(1, 1) = pow(0.5 * sigma_ay, 2);
        Q_(2, 2) = pow(sigma_ax, 2);
        Q_(3, 3) = pow(sigma_ay, 2);

        // Measurement noise (GPS)
        double sigma_gps_pos = 3.0;  // GPS position noise standard deviation (m)
        R_(0, 0) = pow(sigma_gps_pos, 2);
        R_(1, 1) = pow(sigma_gps_pos, 2);

        x_.setZero();
        P_ *= 1000.0;  // Large initial uncertainty
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double dt = 0.01;  // Example time step for IMU data (100 Hz)

        // Update state transition matrix F_ based on time elapsed
        F_(0, 2) = dt;
        F_(1, 3) = dt;

        // Predict step
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // Publish the estimate
        publish_estimate();
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Measurement vector
        Eigen::VectorXd z(2);
        z << msg->latitude, msg->longitude;

        // Update step of the Kalman filter
        Eigen::VectorXd y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;

        // Publish the estimate
        publish_estimate();
    }

    void publish_estimate()
    {
        std_msgs::msg::Float64MultiArray estimate_msg;
        estimate_msg.data = {x_(0), x_(1)};
        estimate_publisher_->publish(estimate_msg);
    }

    void baseline_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // You can log or handle the baseline data here if needed
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr baseline_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr estimate_publisher_;

    // Kalman filter variables
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
