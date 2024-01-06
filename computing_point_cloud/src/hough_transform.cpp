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
#include "std_msgs/msg/float32_multi_array.hpp"


using std::placeholders::_1;
using std::bind;

class Hough_transform : public rclcpp::Node
{
public:
    Hough_transform()
        : Node("hough_transform")
    {
        subscriber_probabilities_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/probabilities", 10, std::bind(&Hough_transform::probability_callback, this, std::placeholders::_1));
        subscription_STL_PC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/STL_point_cloud", 10, bind(&Hough_transform::STLCallback, this, _1));
        subscription_augmented_PC = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/augmented_point_cloud", 10, bind(&Hough_transform::augmented_PC_callback, this, _1));

        publisher_HT = this->create_publisher<sensor_msgs::msg::PointCloud2>("/hough_transform", 10);
    }

private:
    void augmented_PC_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    if (!current_probabilities)
        {
            RCLCPP_WARN(get_logger(), "probabilities not received yet. Skipping point cloud processing.");
            return;
        }
    if (!current_STL)
        {
            RCLCPP_WARN(get_logger(), "STL_PC not received yet. Skipping point cloud processing.");
            return;
        }
    if (
                msg->header.stamp.sec < STL->header.stamp.sec ||
            (msg->header.stamp.sec == STL->header.stamp.sec && msg->header.stamp.nanosec < STL->header.stamp.nanosec)
            )
        {
            RCLCPP_WARN(get_logger(), "augmented point cloud has a lower stamp than STL point cloud");
            return;
        }    

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmentedcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *augmentedcloud);

    //extracting STL point cloud
    pcl_conversions::toPCL(*STL, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSTL(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloudSTL);

    pcl::PointXYZRGB min_point, max_point;
    pcl::getMinMax3D(*cloudSTL, min_point, max_point);

    float mean = calculateMean(probabilities);
    float dx = 0.075, dy = 0.075, dz = 0.075;
    float xHT = min_point.x - dx/2, yHT = min_point.y - dy/2, zHT = min_point.z - dz/2;
    pcl::PointXYZRGB newPoint, oldPoint;
    int current_x = 0;
    int current_y = 0;
    int current_z = 0;

    if (first_iteration){
        index_x = static_cast<int>((max_point.x - min_point.x) / dx) + 2;
        index_y = static_cast<int>((max_point.y - min_point.y) / dy) + 2;
        index_z = static_cast<int>((max_point.z - min_point.z) / dz) + 2;
        count.resize(index_x, std::vector<std::vector<int>>(index_y, std::vector<int>(index_z, 0)));
        HT.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        first_iteration = false;

        zHT = min_point.z - dz/2;
        current_z = 0;
        while (current_z < index_z) {

            yHT = min_point.y - dy/2;
            current_y = 0;
            while (current_y < index_y) {

                xHT = min_point.x - dx/2;
                current_x = 0;
                while (current_x < index_x) {
                    newPoint.r = 255;
                    newPoint.g = 255;
                    newPoint.b = 255;
                    newPoint.x = xHT + dx/2;
                    newPoint.y = yHT + dy/2;
                    newPoint.z = zHT + dz/2;

                    // Add the new point to the PointCloud
                    HT->points.push_back(newPoint);

                    xHT = xHT + dx;
                    current_x++;
                } 

                yHT = yHT + dy;
                current_y++;
            } 

            zHT = zHT + dz;
            current_z++;
        }
    }


    zHT = min_point.z - dz/2;
    current_z = 0;
    while (current_z < index_z) {

        yHT = min_point.y - dy/2;
        current_y = 0;
        while (current_y < index_y) {

            xHT = min_point.x - dx/2;
            current_x = 0;
            while (current_x < index_x) {

                HT->points[current_x + current_y * index_x + current_z * index_x * index_y].x = xHT + dx/2;
                HT->points[current_x + current_y * index_x + current_z * index_x * index_y].y = yHT + dy/2;
                HT->points[current_x + current_y * index_x + current_z * index_x * index_y].z = zHT + dz/2;

                oldPoint = HT->points[current_x + current_y * index_x + current_z * index_x * index_y];

                for (std::size_t i = 0; i < augmentedcloud->points.size(); ++i) {
                    bool condition1 = (augmentedcloud->points[i].x > xHT) && (augmentedcloud->points[i].x < xHT + dx);
                    bool condition2 = (augmentedcloud->points[i].y > yHT) && (augmentedcloud->points[i].y < yHT + dy);
                    bool condition3 = (augmentedcloud->points[i].z > zHT) && (augmentedcloud->points[i].z < zHT + dz);

                    if (condition1 && condition2 && condition3) {

                        oldPoint.rgb = HT->points[current_x + current_y * index_x + current_z * index_x * index_y].rgb;

                        // bad point in case probabilities[i] - mean < 0
                        if (probabilities[i] - mean < 0) {
                            // yellow 255, 255, 0
                            // from yellow to red
                            newPoint.r = 255;
                            newPoint.g = ((probabilities[i] - mean) / mean + 1) * 255;
                            newPoint.b = 0;

                        } else {
                            // from yellow to green
                            newPoint.r = (1 - (probabilities[i] - mean) / mean) * 255;
                            newPoint.g = 255;
                            newPoint.b = 0;
                        }

                        // moving average 
                        newPoint.r = (count[current_x][current_y][current_z] * oldPoint.r + newPoint.r) /
                                    (count[current_x][current_y][current_z] + 1);
                        newPoint.g = (count[current_x][current_y][current_z] * oldPoint.g + newPoint.g) /
                                    (count[current_x][current_y][current_z] + 1);
                        // in case the point is in the range of the parameter space, increase the counter
                        count[current_x][current_y][current_z] += 1; 
    
                        HT->points[current_x + current_y * index_x + current_z * index_x * index_y].rgb = newPoint.rgb;
                    }

                }

                xHT = xHT + dx;
                current_x++;
            } 
            yHT = yHT + dy;
            current_y++;
        } 
        zHT = zHT + dz;
        current_z++;
    }

    // Convert back to PointCloud2
    pcl::toPCLPointCloud2(*HT, pcl_pc2);
    auto HT_PC = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::fromPCL(pcl_pc2, *HT_PC);

    HT_PC->header = PC_header;

    publisher_HT->publish(*HT_PC); 
    
    //finishing to process the old STL point cloud
    current_STL = false;
    current_probabilities = false;

    }

    void probability_callback(const std_msgs::msg::Float32MultiArray msg)
    {

        if (!current_probabilities){
            probabilities = msg.data;
            current_probabilities = true;
        }
        
    }

    void STLCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!current_STL){
            STL = msg;
            current_STL = true;
            PC_header = msg->header;
        }
    }


    float calculateMean(const std::vector<float>& probabilities) {
        float sum = std::accumulate(std::begin(probabilities), std::end(probabilities), 0.0f);
        return sum / static_cast<float>(probabilities.size());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr HT;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_STL_PC, subscription_augmented_PC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_HT;
    sensor_msgs::msg::PointCloud2::SharedPtr STL;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_probabilities_;
    bool current_STL = false, current_probabilities = false, first_iteration = true;
    std_msgs::msg::Header PC_header;
    std::vector<float> probabilities;
    std::vector<std::vector<std::vector<bool>>> pointDefined;
    std::vector<std::vector<std::vector<int>>> count;
    int index_x, index_y, index_z;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Hough_transform>());
    rclcpp::shutdown();
    return 0;
}

//                        std::cout << "\ncurrent x: " << current_x << "\n" << std::endl;
//                        std::cout << "\ncurrent y: " << current_y << "\n" << std::endl;
//                        std::cout << "\ncurrent z: " << current_z << "\n" << std::endl;
//                        std::cout << "\nindex x: " << index_x << "\n" << std::endl;
//                        std::cout << "\nindex y: " << index_y << "\n" << std::endl;
//                        std::cout << "\nindex z: " << index_z << "\n" << std::endl;
//                        std::cout << "\npoints size: " << HT->points.size() << "\n" << std::endl;
//                        std::cout << "\ncurrent index: " << current_x + current_y * index_x + current_z * index_x * index_y << "\n" << std::endl;
//                        std::cout << "\ncurrent point: " << HT->points[current_x + current_y * index_x + current_z * index_x * index_y] << "\n" << std::endl;
//                        oldPoint = HT->points[current_x + current_y * index_x + current_z * index_x * index_y];