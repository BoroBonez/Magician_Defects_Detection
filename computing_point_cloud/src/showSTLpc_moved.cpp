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


        this->declare_parameter<double>("angle_x", 0.0);
        this->declare_parameter<double>("angle_y", 0.0);
        this->declare_parameter<double>("angle_z", 0.0);

        this->declare_parameter<double>("x", 5);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("z", 3);

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSTL(new pcl::PointCloud<pcl::PointXYZ>);
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

        // Compute distance
        /*
        float min_distance = std::numeric_limits<float>::max();

        for (std::size_t i = 0; i < cloudSTL->points.size(); ++i) {
            std::size_t min_d_point_position = NULL;
            std::size_t j = 0;

            for (const auto& pointPC : cloudPC->points) {

                float distance = pcl::euclideanDistance(pointPC, cloudSTL->points[i]);

                if (std::min(min_distance, distance) == distance){
                    min_distance = distance;
                    min_d_point_position = j;
                }
                j++;
            }

            cloudSTL->points[i] = cloudPC->points[min_d_point_position];

        }
        */

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

    void scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double scale_factor) {
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            cloud->points[i].x *= scale_factor;
            cloud->points[i].y *= scale_factor;
            cloud->points[i].z *= scale_factor;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool current_pose_received_ = false;
    //space where is possible to find the object
    float imin, imax, jmin, jmax, kmin, kmax;
    float step = 0.1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pc_stl_relationship>());
    rclcpp::shutdown();
    return 0;
}
