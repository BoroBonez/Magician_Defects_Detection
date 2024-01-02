#include <memory>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

using std::placeholders::_1;
using std::bind;

class Point_cloud_sub : public rclcpp::Node
{
public:
    Point_cloud_sub()
        : Node("point_cloud_sub")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10, bind(&Point_cloud_sub::topic_callback, this, _1));
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/average_quaternions/camera", rclcpp::SensorDataQoS(), bind(&Point_cloud_sub::poseCallback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/new_point_cloud", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!current_pose_received_)
        {
            RCLCPP_WARN(get_logger(), "Pose not received yet. Skipping point cloud processing.");
            return;
        }

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // Original pose
          //translation
        Eigen::Vector3d original_position(0, 0, 0);
          //rotation
        Eigen::Affine3d original_rotation = Eigen::Affine3d::Identity();
        original_rotation *= Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY())
                          *  Eigen::AngleAxisd(+M_PI/2, Eigen::Vector3d::UnitZ())
                          *  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond original_orientation_q( Eigen::Quaterniond(original_rotation.rotation()) );
  
        // Define your transformation matrix
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
          //translation
        Eigen::Vector3d translation(current_pose_->pose.position.z, current_pose_->pose.position.x, current_pose_->pose.position.y);
          //rotation
        Eigen::Quaterniond rotation(current_pose_->pose.orientation.w, current_pose_->pose.orientation.x, current_pose_->pose.orientation.y, current_pose_->pose.orientation.z);
        // Apply the transformation to the original pose
        ((transform.translate(original_position)).translate(translation));
        ((transform.rotate(original_orientation_q)).rotate(rotation));

        pcl::transformPointCloud(*cloud, *cloud, transform);

        // Convert back to PointCloud2
        pcl::toPCLPointCloud2(*cloud, pcl_pc2);
        auto new_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg);

        new_point_cloud_msg->header = msg->header;
        new_point_cloud_msg->header.frame_id = "map";

        publisher_->publish(*new_point_cloud_msg);

        //finishing to compute the previous pose
        current_pose_received_ = false;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        if (!current_pose_received_){
            current_pose_ = pose_msg;
            current_pose_received_ = true;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    bool current_pose_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Point_cloud_sub>());
    rclcpp::shutdown();
    return 0;
}
