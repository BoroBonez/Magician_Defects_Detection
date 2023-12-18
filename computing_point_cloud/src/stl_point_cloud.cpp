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
            "/optiTrack/poseSTL", rclcpp::SensorDataQoS(), bind(&Stl_point_cloud::poseCallback, this, _1));

        publisher_STL = this->create_publisher<sensor_msgs::msg::PointCloud2>("/STL_point_cloud", 10);


        this->declare_parameter<double>("angle_x", 0.0);
        this->declare_parameter<double>("angle_y", 0.0);
        this->declare_parameter<double>("angle_z", 0.0);

        this->declare_parameter<double>("x", 0.0);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("z", 0.0);

    }

private:

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        // Extract vertices from the STL mesh
        pcl::PolygonMesh stl_mesh;
        pcl::io::loadPolygonFileSTL("/home/robotics/ros2_ws_magician/src/display_urdf/stl/mattonella1.stl", stl_mesh);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSTL(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(stl_mesh.cloud, *cloudSTL);
        scalePointCloud(cloudSTL, 0.001);

        //check the values of defects1.urdf
        double roll;
        double pitch;  
        double yaw;      
        double x_translation;
        double y_translation;
        double z_translation; 

        this->get_parameter("angle_x", roll);
        this->get_parameter("angle_y", pitch);
        this->get_parameter("angle_z", yaw);
        this->get_parameter("x", x_translation);
        this->get_parameter("y", y_translation);
        this->get_parameter("z", z_translation);

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        //static rotation 
        transform *= Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::Vector3d translation(x_translation,
                                    y_translation,
                                    z_translation);

        //static translation
        transform.translate(translation);

        //dynamic
        Eigen::Quaterniond rotation((*pose_msg).pose.orientation.w,
                                    (*pose_msg).pose.orientation.x,
                                    (*pose_msg).pose.orientation.y,
                                    (*pose_msg).pose.orientation.z);
        translation.x() = (*pose_msg).pose.position.x;
        translation.y() = (*pose_msg).pose.position.y;
        translation.z() = (*pose_msg).pose.position.z;
        
        transform.translate(translation);
        transform.rotate(rotation);

        pcl::transformPointCloud(*cloudSTL, *cloudSTL, transform);

        // display STL
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(*cloudSTL, pcl_pc2);
        auto new_point_cloud_msg_STL = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg_STL);

        new_point_cloud_msg_STL->header.stamp = pose_msg->header.stamp;
        new_point_cloud_msg_STL->header.frame_id = "STL_pc";
        
        publisher_STL->publish(*new_point_cloud_msg_STL);

        rate.sleep();
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
    rclcpp::Rate rate{1}; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stl_point_cloud>());
    rclcpp::shutdown();
    return 0;
}
