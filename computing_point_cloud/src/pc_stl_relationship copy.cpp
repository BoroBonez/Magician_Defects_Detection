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
//#include <pcl/io/file_io.h>
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
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        
        if (!current_pose_received_)
        {
            RCLCPP_WARN(get_logger(), "Pose not received yet. Skipping point cloud processing.");
            return;
        }

        //check the values of defects1.urdf
        double roll = 0.0;
        double pitch = 3.14159;  
        double yaw = 0.0;        
        double x_translation = -0.150;  
        double y_translation = 0.0;     
        double z_translation = 0.019;   

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);


        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        //rotation
        transform *= Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::Vector3d translation(x_translation,
                                    y_translation,
                                    z_translation);

        //translation
        transform.translate(translation);

        pcl::transformPointCloud(*cloud, *cloud, transform);

        pcl::PointXYZRGB min_point, max_point;
        pcl::getMinMax3D(*cloud, min_point, max_point);
        imin = min_point.x;
        jmin = min_point.y;
        kmin = min_point.z;
        imax = max_point.x;
        jmax = max_point.y;
        kmax = max_point.z;
        // Iterate over each point in the cloud
        for (std::size_t i = 0; i < cloud->points.size(); ++i) {

            pcl::PointXYZRGB pre_processing_point = cloud->points[i];
            //pre_processing_point is set equals to post_processing_point just to copy the RGB in the new point, the XYZ values are modified in this loop
            pcl::PointXYZRGB post_processing_point = pre_processing_point;

            // Access individual components (x, y, z)
            float x = pre_processing_point.x;
            float y = pre_processing_point.y;
            float z = pre_processing_point.z;

            pS_P(x, y, z);
        }


        pcl::PolygonMesh stl_mesh;
        //pcl::io::loadPolygonFileSTL("C://home/robotics/ros2_ws_magician/src/display_urdf/stl/mattonella1.stl", stl_mesh);






        // Convert back to PointCloud2
        pcl::toPCLPointCloud2(*cloud, pcl_pc2);
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

    Eigen::Vector3f pS_P(float x, float y, float z) {
        float p[(int)((imax - imin) / step)][(int)((jmax - jmin) / step)][(int)((kmax - kmin) / step)];


        for(float i = imin; i < imax; i += step) {
            for(float j = jmin; j < jmax; j += step) {
                for(float k = kmin; k < kmax; k += step) {
                    //pP_S() * pS() / pP();
                    std::cout<<"ciao"<<std::endl;
                }
            }
        }
            
        return Eigen::Vector3f(x, y, z);
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
