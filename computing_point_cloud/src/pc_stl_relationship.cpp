#include <memory>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <iostream>
//#include <pcl/io/stl_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <stdio.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

using std::placeholders::_1;
using std::bind;

class Pc_stl_relationship : public rclcpp::Node
{
public:
    Pc_stl_relationship()
        : Node("pc_stl_relationship")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/new_point_cloud", 10, bind(&Pc_stl_relationship::topic_callback, this, _1));
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/optiTrack/poseSTL", rclcpp::SensorDataQoS(), bind(&Pc_stl_relationship::poseCallback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/augmented_point_cloud", 10);
        publisher_STL = this->create_publisher<sensor_msgs::msg::PointCloud2>("/STL_point_cloud", 10);


        this->declare_parameter<double>("angle_x", 0.0);
        this->declare_parameter<double>("angle_y", 0.0);
        this->declare_parameter<double>("angle_z", 0.0);

        this->declare_parameter<double>("x", -0.9);
        this->declare_parameter<double>("y", 0.7);
        this->declare_parameter<double>("z", 0.95);

    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        
        if (!current_pose_received_)
        {
            RCLCPP_WARN(get_logger(), "Pose not received yet. Skipping point cloud processing.");
            return;
        }

        //extracting point cloud message
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPC(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudPC);

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
        Eigen::Quaterniond rotation(current_pose_.pose.orientation.w,
                                    current_pose_.pose.orientation.x,
                                    current_pose_.pose.orientation.y,
                                    current_pose_.pose.orientation.z);
        translation.x() = current_pose_.pose.position.x;
        translation.y() = current_pose_.pose.position.y;
        translation.z() = current_pose_.pose.position.z;
        
        transform.translate(translation);
        transform.rotate(rotation);

        pcl::transformPointCloud(*cloudSTL, *cloudSTL, transform);

        // display STL
        pcl::toPCLPointCloud2(*cloudSTL, pcl_pc2);
        auto new_point_cloud_msg_STL = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg_STL);

        new_point_cloud_msg_STL->header = msg->header;

        publisher_STL->publish(*new_point_cloud_msg_STL);

        //keep just the points in the desired region
        pcl::PointXYZRGB min_point, max_point;
        pcl::getMinMax3D(*cloudSTL, min_point, max_point);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        float step = 0.0;

        pass.setInputCloud(cloudPC);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_point.x - step, max_point.x + step);
        pass.filter(*cloudPC);

        pass.setInputCloud(cloudPC);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(min_point.y - step, max_point.y + step);
        pass.filter(*cloudPC);

        pass.setInputCloud(cloudPC);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_point.z - step, max_point.z + step);
        pass.filter(*cloudPC);

        
        // Compute distance
        for (std::size_t i = 0; i < cloudSTL->points.size(); ++i) {
            cloudSTL->points[i] = min_distance_point_p_pc(cloudSTL->points[i], cloudPC);
        }
        
        // Convert back to PointCloud2
        pcl::toPCLPointCloud2(*cloudSTL, pcl_pc2);
        auto new_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg);

        new_point_cloud_msg->header = msg->header;

        publisher_->publish(*new_point_cloud_msg);

    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        current_pose_ = *pose_msg;
        current_pose_received_ = true;
    }

    void scalePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double scale_factor) {
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            cloud->points[i].x *= scale_factor;
            cloud->points[i].y *= scale_factor;
            cloud->points[i].z *= scale_factor;
        }
    }

    float distance_point_point(pcl::PointXYZRGB point1, pcl::PointXYZRGB point2){
        Eigen::Vector3f diff_vector = Eigen::Vector3f(
            point1.x - point2.x,
            point1.y - point2.y,
            point1.z - point2.z
        );

        return diff_vector.norm();
    }
    //returns the closest point of the point cloud to point1
    pcl::PointXYZRGB min_distance_point_p_pc(pcl::PointXYZRGB point1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
        std::size_t min_d_point_position = 0;
        std::size_t j = 0;
        float min_distance = std::numeric_limits<float>::max();
        for (const auto& pointPC : cloud->points) {
            Eigen::Vector3f diff_vector = Eigen::Vector3f(
                pointPC.x - point1.x,
                pointPC.y - point1.y,
                pointPC.z - point1.z
            );

            float distance = diff_vector.norm();

            if (std::min(min_distance, distance) == distance){
                min_distance = distance;
                min_d_point_position = j;
            }
            j++;
        }
        return cloud->points[min_d_point_position];
    }

    double pS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB point){

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_STL;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool current_pose_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pc_stl_relationship>());
    rclcpp::shutdown();
    return 0;
}
