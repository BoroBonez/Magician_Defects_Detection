#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <Eigen/Dense>

class Average_quaternions : public rclcpp::Node {
public:
  Average_quaternions() : Node("average_quaternions") {
    
    this->declare_parameter<int>("n", 1);
    
    this->declare_parameter<std::string>("input_pose_topic", "/default/average_pose");
    auto topic_param_input = this->get_parameter("input_pose_topic").as_string();

    this->declare_parameter<std::string>("output_pose_topic", "/default/average_pose");
    auto topic_param_output = this->get_parameter("output_pose_topic").as_string();

    // Subscriber
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_param_input, rclcpp::SensorDataQoS(), std::bind(&Average_quaternions::poseCallback, this, std::placeholders::_1));

    // Publisher using the parameter value
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic_param_output, 10);

    M.resize(4, 4);
    average_quaternion.resize(4, 1);
    new_quaternion.resize(4, 1);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // initialization
    n = this->get_parameter("n").as_int();
    new_quaternion << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
    if (n == 1){
      average_quaternion << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
    }

    M = static_cast<double>(n) / (n + 1) * average_quaternion * average_quaternion.transpose() + 
        1.0 / static_cast<double>(n + 1) * new_quaternion * new_quaternion.transpose();

    // eigenvector problem
    Eigen::EigenSolver<Eigen::MatrixXd> solver(M);
    // Get the eigenvalues
    eigenvalues = solver.eigenvalues().real();
    // Find the index of the maximum eigenvalue
    double maxValue = eigenvalues.maxCoeff(&maxIndex);
    // Get the corresponding eigenvector
    eigenvectors = solver.eigenvectors().real();

    average_quaternion_not_scaled = eigenvectors.col(maxIndex);
    average_quaternion = average_quaternion_not_scaled / average_quaternion_not_scaled.norm();

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
    pose_stamped_msg->pose.orientation.w = average_quaternion(0);
    pose_stamped_msg->pose.orientation.x = average_quaternion(1);
    pose_stamped_msg->pose.orientation.y = average_quaternion(2);
    pose_stamped_msg->pose.orientation.z = average_quaternion(3);

    // Publish the PoseStamped message
    pose_publisher_->publish(*pose_stamped_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  double x_mean = 0, y_mean = 0, z_mean = 0;
  Eigen::VectorXd average_quaternion, new_quaternion, eigenvalues, average_quaternion_not_scaled;
  Eigen::MatrixXd M, eigenvectors;
  int n, maxIndex;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Average_quaternions>());
  rclcpp::shutdown();
  return 0;
}
