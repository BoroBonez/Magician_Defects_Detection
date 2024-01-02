#include <memory>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <stdio.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

using std::placeholders::_1;
using std::bind;

class Stl_point_cloud : public rclcpp::Node
{
public:
    Stl_point_cloud()
        : Node("stl_point_cloud")
    {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/average_quaternions/poseSTL", rclcpp::SensorDataQoS(), bind(&Stl_point_cloud::poseCallback, this, _1));

        publisher_STL = this->create_publisher<sensor_msgs::msg::PointCloud2>("/STL_point_cloud", 10);

    }

private:

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        // Extract vertices from the STL mesh
        pcl::PolygonMesh stl_mesh;
        pcl::io::loadPolygonFileSTL("/home/matteo/ros2_ws_git/src/display_with_rviz2/stl/mattonella1.stl", stl_mesh);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSTL(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Assuming you want to assign a constant color, let's say black (0, 0, 0)
        pcl::PointXYZRGB black_color_point;
        black_color_point.r = 0;
        black_color_point.g = 0;
        black_color_point.b = 0;

        // Convert the polygon mesh cloud to a point cloud
        pcl::fromPCLPointCloud2(stl_mesh.cloud, *cloudSTL);

        // Assign black color to all points in the cloud
        for (auto& point : cloudSTL->points) {
            point.r = black_color_point.r;
            point.g = black_color_point.g;
            point.b = black_color_point.b;
        }

        scalePointCloud(cloudSTL, 0.001);


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
        Eigen::Vector3d translation(pose_msg->pose.position.z, pose_msg->pose.position.x, pose_msg->pose.position.y);
          //rotation
        Eigen::Quaterniond rotation(pose_msg->pose.orientation.w, pose_msg->pose.orientation.z, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y);
  
        // Apply the transformation to the original pose
        ((transform.translate(original_position)).translate(translation));
        ((transform.rotate(original_orientation_q)).rotate(rotation));

        pcl::transformPointCloud(*cloudSTL, *cloudSTL, transform);

        // display STL
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*cloudSTL, pcl_pc2);
        auto new_point_cloud_msg_STL = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg_STL);

        new_point_cloud_msg_STL->header.stamp = pose_msg->header.stamp;
        new_point_cloud_msg_STL->header.frame_id = "map";
        
        publisher_STL->publish(*new_point_cloud_msg_STL);
    }

    void scalePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double scale_factor) {
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            cloud->points[i].x *= scale_factor;
            cloud->points[i].y *= scale_factor;
            cloud->points[i].z *= scale_factor;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_STL;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stl_point_cloud>());
    rclcpp::shutdown();
    return 0;
}
