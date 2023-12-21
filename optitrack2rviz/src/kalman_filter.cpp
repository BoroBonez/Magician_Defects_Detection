#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <Eigen/Dense>

class Kalman_filter : public rclcpp::Node {
public:
  Kalman_filter() : Node("kalman_filter") {
    
    this->declare_parameter<int>("n", 1);
    
    this->declare_parameter<std::string>("input_pose_topic", "/default/average_pose");
    auto topic_param_input = this->get_parameter("input_pose_topic").as_string();

    this->declare_parameter<std::string>("output_pose_topic", "/default/average_pose");
    auto topic_param_output = this->get_parameter("output_pose_topic").as_string();

    // Subscriber
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_param_input, rclcpp::SensorDataQoS(), std::bind(&Kalman_filter::poseCallback, this, std::placeholders::_1));

    // Publisher using the parameter value
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic_param_output, 10);

    state_estimated.resize(8, 1);
    state_predicted.resize(8, 1);
    F.resize(8, 8);
    H.resize(5, 8);
    C_e.resize(8, 8);
    K.resize(8, 5);
    z.resize(5, 1);
    S.resize(5, 5);
    z_prediction.resize(5, 1);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    //initialization
    n = this->get_parameter("n").as_int();

    F << 1, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0,
        1.0 / static_cast<double>(n + 1), 0, 0, 0, static_cast<double>(n) / (n + 1), 0, 0, 0,
        0, 1.0 / static_cast<double>(n + 1), 0, 0, 0, static_cast<double>(n) / (n + 1), 0, 0,
        0, 0, 1.0 / static_cast<double>(n + 1), 0, 0, 0, static_cast<double>(n) / (n + 1), 0,
        0, 0, 0, 1.0 / static_cast<double>(n + 1), 0, 0, 0, static_cast<double>(n) / (n + 1);


    if (n == 1){
      state_estimated << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, 0, 0, 0, 0;
      H << 1, 0, 0, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 0, 2, 2, 2, 2;
      C_e = Eigen::MatrixXd::Zero(8, 8);
      C_e.diagonal() << 1, 1, 1, 1, 1, 1, 1, 1;
      C_z = Eigen::MatrixXd::Zero(5, 5);
      C_z.diagonal() << 0.1, 0.1, 0.1, 0.1, 0;
      C_n = Eigen::MatrixXd::Zero(8, 8);
      C_n.diagonal() << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
      //std::cout << "\nn==1\n" << std::endl;

    }

    //prediction
    state_predicted = F * state_estimated;
    //std::cout << "\nthe prediction is: " << state_predicted << std::endl;
    C_p = F * C_e * F.transpose() + C_n;
    //estimation
    S = C_z + H * C_p * H.transpose();
    K = C_p * H.transpose() * S.inverse();
    z << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, 1;
    z_prediction << (H * state_predicted).head(4), std::sqrt( std::pow(msg->pose.orientation.w, 2) + std::pow(msg->pose.orientation.x, 2) + std::pow(msg->pose.orientation.y, 2) + std::pow(msg->pose.orientation.z, 2) );
    state_estimated = state_predicted + K * (z - z_prediction);
    C_e = C_p - K * S * K.transpose();
    //update
    x_mean = (x_mean * n + msg->pose.position.x) / (n + 1);
    y_mean = (y_mean * n + msg->pose.position.y) / (n + 1);
    z_mean = (z_mean * n + msg->pose.position.z) / (n + 1);
    this->set_parameter(rclcpp::Parameter("n", ++n));
    // Create a PoseStamped message
    auto pose_stamped_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    pose_stamped_msg->header = msg->header;
    pose_stamped_msg->pose.position.x = x_mean;
    pose_stamped_msg->pose.position.y = y_mean;
    pose_stamped_msg->pose.position.z = z_mean;
    pose_stamped_msg->pose.orientation.w = state_estimated(4);
    pose_stamped_msg->pose.orientation.x = state_estimated(5);
    pose_stamped_msg->pose.orientation.y = state_estimated(6);
    pose_stamped_msg->pose.orientation.z = state_estimated(7);

    //std::cout << "\nthe norm is: "<< std::sqrt( std::pow(msg->pose.orientation.w, 2) + std::pow(msg->pose.orientation.x, 2) + std::pow(msg->pose.orientation.y, 2) + std::pow(msg->pose.orientation.z, 2) ) << std::endl;
    //std::cout << "\nthe quaternion is: " << state_estimated << std::endl;

    // Publish the PoseStamped message
    pose_publisher_->publish(*pose_stamped_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  double x_mean = 0, y_mean = 0, z_mean = 0;
  Eigen::VectorXd state_estimated, state_predicted, z, z_prediction;
  Eigen::MatrixXd F, H, C_e, C_p, K, S, C_z, C_n;
  int n;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Kalman_filter>());
  rclcpp::shutdown();
  return 0;
}
