#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <Eigen/Dense>

class PoseToTFNode : public rclcpp::Node {
public:
  PoseToTFNode() : Node("pose_to_tf_STL") {
    // Subscriber
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/optiTrack/poseSTL", rclcpp::SensorDataQoS(), std::bind(&PoseToTFNode::poseCallback, this, std::placeholders::_1));


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
      Eigen::Vector3d original_position(0, 0.148, 0.02);
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
  }


  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_{this};  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToTFNode>());
  rclcpp::shutdown();
  return 0;
}
