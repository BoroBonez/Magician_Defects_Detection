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
        publisher_HT = this->create_publisher<sensor_msgs::msg::PointCloud2>("/hough_transform", 10);
        publisher_probabilities = this->create_publisher<std_msgs::msg::Float32MultiArray>("probabilities_topic", 10);
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmentedcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudSTL);
        pcl::fromPCLPointCloud2(pcl_pc2, *augmentedcloud);

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

        std::vector<float> probabilities;
        probabilities.clear();
        for (std::size_t i = 0; i < cloudSTL->points.size(); ++i) {
            //std::cout << "\n\n" << cloudSTL->points.size() << " point lefted to compute\n\n" << std::endl;
            augmentedcloud->points[i] = max_probability_point_p_pc(cloudSTL->points[i], cloudPC, probabilities);
            //cloudSTL->points.erase(cloudSTL->points.begin() + i);
        }
        float mean = calculateMean(probabilities);

        //Hough transform
        float dx = 0.1, dy = 0.1, dz = 0.1;
        float xHT = min_point.x, yHT = min_point.y, zHT = min_point.z;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr HT(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointXYZRGB newPoint, oldPoint;
        int index_x = static_cast<int>((max_point.x - min_point.x) / dx);
        int index_y = static_cast<int>((max_point.y - min_point.y) / dy);
        int index_z = static_cast<int>((max_point.z - min_point.z) / dz);
        std::vector<std::vector<std::vector<int>>> count(index_x, std::vector<std::vector<int>>(index_y, std::vector<int>(index_z, 0)));

        do {
            do {
                do {
                    for (std::size_t i = 0; i < augmentedcloud->points.size(); ++i) {
                        bool condition1 = (augmentedcloud->points[i].x > xHT) && (augmentedcloud->points[i].x < xHT + dx);
                        bool condition2 = (augmentedcloud->points[i].y > yHT) && (augmentedcloud->points[i].y < yHT + dy);
                        bool condition3 = (augmentedcloud->points[i].z > zHT) && (augmentedcloud->points[i].z < zHT + dz);
                        
                        if (condition1 && condition2 && condition3) {
                            // in case the point is in the range of the parameter space, increase the counter
                            int current_x = static_cast<int>((xHT - min_point.x) / dx);
                            int current_y = static_cast<int>((yHT - min_point.y) / dy);
                            int current_z = static_cast<int>((zHT - min_point.z) / dz);

                            //oldPoint = newPoint;

                            // bad point in case probabilities[i] - mean > 0
                            if (probabilities[i] - mean > 0) {
                                // yellow 255, 255, 0
                                // from yellow to red
                                newPoint.r = 255;
                                newPoint.g = (1 - (probabilities[i] - mean) / mean) * 255;
                                newPoint.b = 0;
                            } else {
                                // from yellow to green
                                newPoint.r = (1 - (probabilities[i] - mean) / mean) * 255;
                                newPoint.g = 255;
                                newPoint.b = 0;
                            }

                            // moving average 
/*                             newPoint.r = (count[current_x][current_y][current_z] * oldPoint.r + newPoint.r) /
                                        (count[current_x][current_y][current_z] + 1);
                            newPoint.g = (count[current_x][current_y][current_z] * oldPoint.g + newPoint.g) /
                                        (count[current_x][current_y][current_z] + 1);
                            count[current_x][current_y][current_z] += 1; */

                            break;
                        } else {
                            newPoint.r = 255;
                            newPoint.g = 255;
                            newPoint.b = 255;
                        }
                    }
                    
                    newPoint.x = xHT;
                    newPoint.y = yHT;
                    newPoint.z = zHT;

                    // Add the new point to the PointCloud
                    HT->points.push_back(newPoint);

                    zHT = zHT + dz;
                } while (zHT <= max_point.z);
                zHT = min_point.z;

                yHT = yHT + dy;
            } while (yHT <= max_point.y);
            yHT = min_point.y;

            xHT = xHT + dx;
        } while (xHT <= max_point.x);
        
        // Convert back to PointCloud2
        pcl::toPCLPointCloud2(*augmentedcloud, pcl_pc2);
        auto new_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg);

        new_point_cloud_msg->header = msg->header;

        publisher_->publish(*new_point_cloud_msg);

        // Convert back to PointCloud2
/*         pcl::toPCLPointCloud2(*HT, pcl_pc2);
        auto new_point_cloud_msg_2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg_2);

        new_point_cloud_msg_2->header = msg->header;

        publisher_HT->publish(*new_point_cloud_msg_2); 
        HT.reset(); */

        std_msgs::msg::Float32MultiArray msg_prob;
        msg_prob.data = probabilities;

        publisher_probabilities->publish(msg_prob);

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

    float gaussianPDF(float x, float mean, float std) {
        float exponent = -(std::pow(x - mean, 2) / (2 * std * std));
        return (1.0f / (std * std::sqrt(2 * M_PI))) * std::exp(exponent);
    }

    float multivariateGaussianPDF(const Eigen::Vector3f& X, const Eigen::Vector3f& mean, const Eigen::Matrix3f& covariance) {
        Eigen::Matrix3f covarianceInverse = covariance.inverse();
        float detCovariance = covariance.determinant();

        Eigen::Vector3f diff = X - mean;
        float exponent = -0.5 * diff.transpose() * covarianceInverse * diff;

        return (1.0f / std::pow(2 * M_PI, 1.5) * std::sqrt(detCovariance)) * std::exp(exponent);
    }

    float distance_point_point(pcl::PointXYZRGB point1, pcl::PointXYZRGB point2){
        Eigen::Vector3f diff_vector = Eigen::Vector3f(
            point1.x - point2.x,
            point1.y - point2.y,
            point1.z - point2.z
        );

        return diff_vector.norm();
    }

    float p_of_S(pcl::PointXYZRGB S, pcl::PointXYZRGB P){
        //considering that the STL is not perfectly overlapped we assume a std of 5cm
        float std = 0.05;
        float distance = distance_point_point(S, P);
        return gaussianPDF(distance, 0.0, std);
    }

    float p_of_P_S(pcl::PointXYZRGB P, pcl::PointXYZRGB S){
/*         float X_dist = std::abs(S.x - P.x);
        float Y_dist = std::abs(S.y - P.y);
        float Z_dist = std::abs(S.z - P.z);

        float B = 0.055;
        float f = 0.00188;
        float pixel_s = 1.4e-6;

        float x_pixel = X_dist * f / Z_dist;
        float y_pixel = Y_dist * f / Z_dist;

        float std_Z = std::pow(Z_dist, 2) / (B * f) * pixel_s; 
        // d (X_dist) / d (Z_dist) = x_pixel / f
        // d (X_dist) / d (x_pixel) =  Z_dist / f
        float std_X = (x_pixel / f) * std_Z + (Z_dist / f) * pixel_s;
        float std_Y = (y_pixel / f) * std_Z + (Z_dist / f) * pixel_s; */

        Eigen::Vector3f X(P.x, P.y, P.z);  // Replace with the desired coordinates [x, y, z]
        Eigen::Vector3f mean(S.x, S.y, S.z);  // Replace with the mean vector [mu_x, mu_y, mu_z]

        float std_X = 0.01;
        float std_Y = 0.01;
        float std_Z = 0.01;

        float var_X = std::pow(std_X, 2);
        float var_Y = std::pow(std_Y, 2);
        float var_Z = std::pow(std_Z, 2);

        Eigen::Matrix3f covariance;  // Replace with the desired covariance matrix
        covariance << var_X, 0.0f, 0.0f,
                      0.0f, var_Y, 0.0f,
                      0.0f, 0.0f, var_Z;

        return multivariateGaussianPDF(X, mean, covariance);

    }

/*     float p_of_P(pcl::PointXYZRGB P, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSTL){
        float prob = 0;
        for (const auto& STLpoint : cloudSTL->points) {
            prob = prob + p_of_S(STLpoint, P) * p_of_P_S(P, STLpoint);
        }
        return prob;
    } */

    float p_of_S_P(pcl::PointXYZRGB S, pcl::PointXYZRGB P){
        return p_of_S(S, P) * p_of_P_S(P, S);
    }

    pcl::PointXYZRGB max_probability_point_p_pc(pcl::PointXYZRGB point1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<float>& probabilities) {
        std::size_t max_p_point_position = 0;
        std::size_t j = 0;
        float max_probability = std::numeric_limits<float>::min();

        for (const auto& pointPC : cloud->points) {
            float probability = p_of_S_P(point1, pointPC);

            if (std::max(max_probability, probability) == probability) {
                max_probability = probability;
                max_p_point_position = j;
            }
            j++;
        }

        probabilities.push_back(max_probability);
        return cloud->points[max_p_point_position];
    }

    float calculateMean(const std::vector<float>& probabilities) {
        float sum = std::accumulate(std::begin(probabilities), std::end(probabilities), 0.0f);
        return sum / static_cast<float>(probabilities.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_STL_PC, subscription_camera_PC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_HT;
    sensor_msgs::msg::PointCloud2::SharedPtr STL;
    bool current_STL = false;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_probabilities;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pc_stl_relationship>());
    rclcpp::shutdown();
    return 0;
}
