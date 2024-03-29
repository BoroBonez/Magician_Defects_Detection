#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <Eigen/Dense>

class PoseToTFNode : public rclcpp::Node {
public:
  PoseToTFNode() : Node("pose_to_tf_STL") {

    this->declare_parameter<std::string>("input_pose_topic", "/kalman_filter/poseSTL");
    auto topic_param_output = this->get_parameter("input_pose_topic").as_string();
    // Subscriber
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_param_output, rclcpp::SensorDataQoS(), std::bind(&PoseToTFNode::poseCallback, this, std::placeholders::_1));

    publisher_pose = create_publisher<geometry_msgs::msg::PoseStamped>("/pose_STL_topic", 10);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      // Convert geometry_msgs::PoseStamped to geometry_msgs::TransformStamped
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header = msg->header;
      transform_stamped.header.frame_id = "map";
      transform_stamped.child_frame_id = "defects"; 
      
      // Original pose
        //translation
      Eigen::Vector3d original_position(0.03, 0.1, 0.0);
        //rotation
      Eigen::Affine3d original_rotation = Eigen::Affine3d::Identity();
      original_rotation *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
                        *  Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())
                        *  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond original_orientation_q( Eigen::Quaterniond(original_rotation.rotation()) );

      // Define your transformation matrix
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        //translation
      Eigen::Vector3d translation(msg->pose.position.z, msg->pose.position.x, msg->pose.position.y);
        //rotation
      Eigen::Quaterniond rotation(msg->pose.orientation.w, msg->pose.orientation.z, msg->pose.orientation.x, msg->pose.orientation.y);

      // Apply the transformation to the original pose
      Eigen::Vector3d transformed_position = ((transform.translate(original_position)).translate(translation)).translation();
      Eigen::Quaterniond transformed_orientation( ((transform.rotate(original_orientation_q)).rotate(rotation)).rotation() );
      //Eigen::Vector3d transformed_position = ((transform.translate(original_position))).translation();
      //Eigen::Quaterniond transformed_orientation( ((transform.rotate(original_orientation_q))).rotation() );

      // Fill in the transformed pose
      transform_stamped.transform.translation.x = transformed_position.x();
      transform_stamped.transform.translation.y = transformed_position.y();
      transform_stamped.transform.translation.z = transformed_position.z();

      transform_stamped.transform.rotation.x = transformed_orientation.x();
      transform_stamped.transform.rotation.y = transformed_orientation.y();
      transform_stamped.transform.rotation.z = transformed_orientation.z();
      transform_stamped.transform.rotation.w = transformed_orientation.w();

      // Broadcast the transformed pose
      tf_broadcaster_.sendTransform(transform_stamped);
      auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose_msg->pose.position.x = transformed_position.x();
      pose_msg->pose.position.y = transformed_position.y();
      pose_msg->pose.position.z = transformed_position.z();
      pose_msg->pose.orientation.x = transformed_orientation.x();
      pose_msg->pose.orientation.y = transformed_orientation.y();
      pose_msg->pose.orientation.z = transformed_orientation.z();
      pose_msg->pose.orientation.w = transformed_orientation.w();
      
      pose_msg->header = msg->header;
      publisher_pose->publish(*pose_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_{this};  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToTFNode>());
  rclcpp::shutdown();
  return 0;
}
