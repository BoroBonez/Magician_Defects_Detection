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
        subscription_camera_PC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/new_point_cloud", 10, bind(&Pc_stl_relationship::topic_callback, this, _1));
        subscription_STL_PC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/STL_point_cloud", 10, bind(&Pc_stl_relationship::STLCallback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/augmented_point_cloud", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!current_STL)
        {
            RCLCPP_WARN(get_logger(), "STL_PC not received yet. Skipping point cloud processing.");
            return;
        }
        else if (
                 msg->header.stamp.sec < STL->header.stamp.sec ||
                (msg->header.stamp.sec == STL->header.stamp.sec && msg->header.stamp.nanosec < STL->header.stamp.nanosec)
                )
        {
            RCLCPP_WARN(get_logger(), "camera point cloud has a lower stamp than STL point cloud");
            //std::cout << "\nCAMERA sec: " << msg->header.stamp.sec << "\nSTL sec: " << STL->header.stamp.sec << std::endl;
            return;
        }
        //extracting camera point cloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPC(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudPC);

        //extracting STL point cloud
        pcl_conversions::toPCL(*STL, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSTL(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudSTL);

        //keep just the points in the desired region
        pcl::PointXYZRGB min_point, max_point;
        pcl::getMinMax3D(*cloudSTL, min_point, max_point);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        float step = 0.01;

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

        
        // Compute distance and replace each point of cloudSTL with the closest point of cloudPC
        for (std::size_t i = 0; i < cloudSTL->points.size(); ++i) {
           cloudSTL->points[i] = min_distance_point_p_pc(cloudSTL->points[i], cloudPC);
        }
        
        // Convert back to PointCloud2
        pcl::toPCLPointCloud2(*cloudSTL, pcl_pc2);
        auto new_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg);

        new_point_cloud_msg->header = msg->header;

        publisher_->publish(*new_point_cloud_msg);
        //finishing to process the old STL point cloud
        current_STL = false;

    }

    void STLCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!current_STL){
            STL = msg;
            current_STL = true;
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_STL_PC, subscription_camera_PC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_STL;
    sensor_msgs::msg::PointCloud2::SharedPtr STL;
    bool current_STL = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pc_stl_relationship>());
    rclcpp::shutdown();
    return 0;
}
