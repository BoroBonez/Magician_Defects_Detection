#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <Eigen/Dense>

class Kalman_filter : public rclcpp::Node {
public:
  Kalman_filter() : Node("kalman_filter") {
    
    this->declare_parameter<std::string>("average_pose_topic", "/default/average_pose");
    auto topic_param = this->get_parameter("average_pose_topic").as_string();

    // Subscriber
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/optiTrack/poseSTL", rclcpp::SensorDataQoS(), std::bind(&Kalman_filter::poseCallback, this, std::placeholders::_1));

    // Publisher using the parameter value
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic_param, 10);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Create a PoseStamped message
    auto pose_stamped_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    pose_stamped_msg->header = msg->header;
    pose_stamped_msg->pose.position.x = 1.0;
    pose_stamped_msg->pose.position.y = 2.0;
    pose_stamped_msg->pose.position.z = 3.0;
    pose_stamped_msg->pose.orientation.x = 0.0;
    pose_stamped_msg->pose.orientation.y = 0.0;
    pose_stamped_msg->pose.orientation.z = 0.0;
    pose_stamped_msg->pose.orientation.w = 1.0;

    // Publish the PoseStamped message
    pose_publisher_->publish(*pose_stamped_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Kalman_filter>());
  rclcpp::shutdown();
  return 0;
}
