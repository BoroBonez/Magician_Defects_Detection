#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//#include <pcl/io/stl_io.h>
#include <Eigen/Core>
#include <fstream>
#include <igl/readSTL.h>
#include <igl/unproject.h>
#include <igl/ray_mesh_intersect.h>
#include "igl/Hit.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "defines.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using std::bind;
using std::placeholders::_1;

class Triangle {
    public:
        // Constructor
        Triangle(float prob) : probability(prob) {
        }

        void print_probability() const {
            std::cout << "\nprobability: " << probability << std::endl;
        }

        void print_error() {
            if(error)
                std::cout << "\nerror: " << error << std::endl;
            else 
                std::cout << "\nthe error is not available yet" << std::endl;
        }

        void error_estimation() {

        }

        float probability, error, normal_dist_camera;
};

class Mesh {
    public:
        Mesh(std::size_t size, Eigen::MatrixXd V_copy, Eigen::MatrixXi F_copy) : n_of_triangles(size), V(V_copy), F(F_copy) {
            //initialization of each triangles
            triangles.reserve(size);
            for (std::size_t  i = 0; i < size; ++i) {
                triangles.push_back(std::make_unique<Triangle>(-1.0f));
            }
            normal_to_each_vertex.resize(V.rows());

            //initialization of the normal direction of each vertex
            std::vector<int> connectedVertices_id;
            for (std::size_t  i = 0; i < (std::size_t )V.rows(); ++i) {
                findConnectedVertices(i, F, connectedVertices_id);
                Eigen::MatrixXd A(connectedVertices_id.size(), 3);

                for (std::size_t m = 0; m < connectedVertices_id.size(); ++m) {
                        A.row(m) = V.row(connectedVertices_id[m]) - V.row(i);
                }

                //PCA
                Eigen::MatrixXd Covariance = A.transpose() * A;
                Eigen::EigenSolver<Eigen::MatrixXd> solver(Covariance);
                Eigen::MatrixXd eigenvectors = solver.eigenvectors().real();
                Eigen::VectorXd eigenvalues = solver.eigenvalues().real();
                int minEigenvalueIndex;
                eigenvalues.minCoeff(&minEigenvalueIndex);
                // Get the corresponding eigenvector
                Eigen::Vector3d minEigenvector = eigenvectors.col(minEigenvalueIndex);
                normal_to_each_vertex[i] = minEigenvector;
            }

        }

        float calculateMean() {
            size_t size = 0;
            float sum = 0;
            for (std::size_t i = 0; i < n_of_triangles; ++i) {
                if (triangles[i]->probability > 0){
                    size++;
                    sum += triangles[i]->probability;
                }
            }
            return sum / size;
        }

        float min_probability(){
            float min = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < n_of_triangles; ++i) {
                if (triangles[i]->probability > 0){
                    if (triangles[i]->probability < min){
                        min = triangles[i]->probability;
                    }
                }
            }
            return min;
        }

        float max_probability(){
            float max = std::numeric_limits<float>::lowest();
            for (std::size_t i = 0; i < (std::size_t)n_of_triangles; ++i) {
                if (triangles[i]->probability > 0){
                    if (triangles[i]->probability > max){
                        max = triangles[i]->probability;
                    }
                }
            }
            return max;
        }

        void print_probabilities(){
            std::cout << "\n" << std::endl;
            for (std::size_t i = 0; i < n_of_triangles; ++i) {
                triangles[i]->print_probability();
                std::cout << "\n" << std::endl;
            }
        }

        std::vector<std::unique_ptr<Triangle>> triangles;
        std::size_t n_of_triangles;
        std::vector<Eigen::Vector3d> normal_to_each_vertex;
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;

    private:
        void findConnectedVertices(int selectedVertexID, const Eigen::MatrixXi& F, std::vector<int>& connectedVertices) {
            connectedVertices.clear();
            
            // Iterate through each face
            for (int i = 0; i < F.rows(); ++i) {
                // Check if the selected vertex is part of the current face
                if (F(i, 0) == selectedVertexID || F(i, 1) == selectedVertexID || F(i, 2) == selectedVertexID) {
                    // Add all vertices of the current face to the connectedVertices vector
                    connectedVertices.push_back(F(i, 0));
                    connectedVertices.push_back(F(i, 1));
                    connectedVertices.push_back(F(i, 2));
                }
            }

            // Remove duplicates from the connectedVertices vector
            std::sort(connectedVertices.begin(), connectedVertices.end());
            // erase from an index to another index
            connectedVertices.erase(std::unique(connectedVertices.begin(), connectedVertices.end()), connectedVertices.end());
            connectedVertices.erase(std::remove(connectedVertices.begin(), connectedVertices.end(), selectedVertexID), connectedVertices.end());
        }
};

class Raycast : public rclcpp::Node {
public:
    Raycast() : Node("raycast") {
        subscription_camera_PC =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        "/new_point_cloud", 10, bind(&Raycast::topic_callback, this, _1)
                );
        pose_subscriber_camera = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/pose_camera_topic",
                rclcpp::SensorDataQoS(),
                std::bind(&Raycast::cameraCallback, this, std::placeholders::_1)
        );
        pose_subscriber_STL = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/pose_STL_topic",
                rclcpp::SensorDataQoS(),
                std::bind(&Raycast::STLCallback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/augmented_point_cloud", 10
        );
        publisher_with_probabilities =
                this->create_publisher<std_msgs::msg::Float32MultiArray>(
                        "/probabilities", 10
                );
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        //check to synchronize
        if (!current_STL) {
            RCLCPP_WARN(
                    get_logger(),
                    "STL pose not received yet. Skipping point cloud processing."
            );
            return;
        }
        if (msg->header.stamp.sec < STL_pose->header.stamp.sec ||
            (msg->header.stamp.sec == STL_pose->header.stamp.sec &&
             msg->header.stamp.nanosec < STL_pose->header.stamp.nanosec)) {
            RCLCPP_WARN(
                    get_logger(),
                    "camera point cloud has a lower stamp than STL point cloud"
            );
            return;
        }
        if (!current_camera_pose) {
            RCLCPP_WARN(
                    get_logger(),
                    "camera pose not received yet. Skipping point cloud processing."
            );
            return;
        }
        if (msg->header.stamp.sec < camera_pose->header.stamp.sec ||
            (msg->header.stamp.sec == camera_pose->header.stamp.sec &&
             msg->header.stamp.nanosec < camera_pose->header.stamp.nanosec)) {
            RCLCPP_WARN(
                    get_logger(),
                    "camera point cloud has a lower stamp than camera pose"
            );
            return;
        }

        // extracting camera point cloud from message
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPC(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudPC);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmentedcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // extracting STL point cloud
        std::string filename = std::string("/home/matteo/ros2_ws_git/src/display_with_rviz2/stl/mattonella1.stl");
        std::ifstream file;
        file.open(filename);
        // V has the coordinates on the column
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        Eigen::MatrixXd N;
        if (!igl::readSTL(file, V, F, N)) {
            std::cout << "Failed parsing STL file" << std::endl;
            file.close();  // Close the file before returning
            return;        // Return an error code or handle the error accordingly
        }
        scalePointCloud(V, 0.001);
        file.close();

        // Extract translation and rotation information from PoseStamped
        Eigen::Vector3d translation(
                STL_pose->pose.position.x,
                STL_pose->pose.position.y,
                STL_pose->pose.position.z
        );
        Eigen::Quaterniond rotation(
                STL_pose->pose.orientation.w,
                STL_pose->pose.orientation.x,
                STL_pose->pose.orientation.y,
                STL_pose->pose.orientation.z
        );

        // Perform rotation
        Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
        V = (rotation_matrix * V.transpose()).transpose();

        // Perform translation
        V.rowwise() += translation.transpose();

        // keep just the points in the desired region
        Eigen::Vector3d min_point, max_point;
        min_point = V.colwise().minCoeff();
        max_point = V.colwise().maxCoeff();

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        float step = 0.01;

        pass.setInputCloud(cloudPC);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_point.x() - step, max_point.x() + step);
        pass.filter(*cloudPC);

        pass.setInputCloud(cloudPC);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(min_point.y() - step, max_point.y() + step);
        pass.filter(*cloudPC);

        pass.setInputCloud(cloudPC);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_point.z() - step, max_point.z() + step);
        pass.filter(*cloudPC);

        //initialization
        if (first_iteration){
            mesh_ptr = std::make_unique<Mesh>(F.rows(), V, F);
            count.resize(F.rows(), 0);
            first_iteration = false;
        }
        
        //perform raycast
        std::vector<int> id_point_at_index;
        id_point_at_index.clear();
        
        for (std::size_t i = 0; i < cloudPC->points.size(); ++i) {
                Eigen::Vector3d origin(
                        camera_pose->pose.position.x,
                        camera_pose->pose.position.y,
                        camera_pose->pose.position.z
                );
                Eigen::Vector3d direction(
                        cloudPC->points[i].x - origin(0),
                        cloudPC->points[i].y - origin(1),
                        cloudPC->points[i].z - origin(2)
                );
                direction = direction / direction.norm();

                std::vector<igl::Hit> hits;
                igl::ray_mesh_intersect(origin, direction, V, F, hits);
                if (hits.size() > 0) {
                        // Intersection found
                        pcl::PointXYZRGB p;
                        p.x = origin.x() + hits.front().t * direction.x();
                        p.y = origin.y() + hits.front().t * direction.y();
                        p.z = origin.z() + hits.front().t * direction.z();

                        mesh_ptr->triangles[hits.front().id]->probability = ( mesh_ptr->triangles[hits.front().id]->probability * count[hits.front().id] + p_of_P_S(cloudPC->points[i], p) ) / ( count[hits.front().id] + 1);
                        //mesh_ptr->triangles[hits.front().id]->normal_dist_camera = 
                        augmentedcloud->points.push_back(p);
                        id_point_at_index.push_back(hits.front().id);
                        count[hits.front().id]++;
                } 

        }

        //set the new color to show the heatmap
        float mean = mesh_ptr->calculateMean();
        float lowest_prob = mesh_ptr->min_probability();
        float highest_prob = mesh_ptr->max_probability(); 

        //std::cout << "\n\nthe mean is: " << mean << "\nthe lowest_prob is: " << lowest_prob << "\nthe highest_prob is: " << highest_prob << std::endl;

        for (std::size_t i = 0; i < augmentedcloud->points.size(); ++i) {

                int id = id_point_at_index[i];

                if ((mesh_ptr->triangles[id])->probability - mean < 0) {
                        // yellow 255, 255, 0
                        // from yellow to red
                        augmentedcloud->points[i].r = 255;
                        augmentedcloud->points[i].g = ( 1 - ((mesh_ptr->triangles[id])->probability - mean)/(lowest_prob - mean) ) * 255;
                        augmentedcloud->points[i].b = 0;

                } else {
                        // from yellow to green
                        augmentedcloud->points[i].r = ( 1 - ((mesh_ptr->triangles[id])->probability - mean)/(highest_prob - mean) ) * 255;
                        augmentedcloud->points[i].g = 255;
                        augmentedcloud->points[i].b = 0;
                }

        }

        // Convert back to PointCloud2
        pcl::toPCLPointCloud2(*augmentedcloud, pcl_pc2);
        auto new_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg);

        new_point_cloud_msg->header = msg->header;

        publisher_->publish(*new_point_cloud_msg);

        // finishing to process the old STL point cloud
        current_STL         = false;
        current_camera_pose = false;
    }

    void cameraCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!current_camera_pose) {
            camera_pose         = msg;
            current_camera_pose = true;
        }
    }

    void STLCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!current_STL) {
            STL_pose    = msg;
            current_STL = true;
        }
    }

    // other function
    void scalePointCloud(Eigen::MatrixXd& cloud, double scale_factor) {
        for (std::size_t i = 0; i < (std::size_t)cloud.rows(); ++i) { cloud.row(i) *= scale_factor; }
    }

    float gaussianPDF(float x, float mean, float std) {
        float exponent = -(std::pow(x - mean, 2) / (2 * std * std));
        return (1.0f / (std * std::sqrt(2 * M_PI))) * std::exp(exponent);
    }

    float distance_point_point(pcl::PointXYZRGB point1, pcl::PointXYZRGB point2){
        Eigen::VectorXf diff_vector(3); 
        diff_vector << point1.x - point2.x,
                       point1.y - point2.y,
                       point1.z - point2.z;

        return diff_vector.norm();
    }

    float p_of_P_S(pcl::PointXYZRGB P, pcl::PointXYZRGB S){
        //extract camera's z axis direction 
        const auto& orientation = camera_pose->pose.orientation;
        Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
        Eigen::Vector3d axis_direction;
        axis_direction = rotation_matrix.col(2);
        //distance component along that direction
        Eigen::Vector3d from_camera_to_point_vector(
                        S.x - camera_pose->pose.position.x,
                        S.y - camera_pose->pose.position.y,
                        S.z - camera_pose->pose.position.z
                );

        float Z_dist = from_camera_to_point_vector.dot(axis_direction);
        float std = 0.0184 * std::exp(0.2106 * Z_dist);

        float distance = distance_point_point(S, P);
        return gaussianPDF(distance, 0.0, std);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_camera_PC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_with_probabilities;
    bool current_STL = false, current_camera_pose = false, first_iteration = true;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_camera, pose_subscriber_STL;
    geometry_msgs::msg::PoseStamped::SharedPtr camera_pose, STL_pose;
    std::vector<int> count;
    std::unique_ptr<Mesh> mesh_ptr; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Raycast>());
    rclcpp::shutdown();
    return 0;
}


/*
    float multivariateGaussianPDF(const Eigen::Vector3f& X, const Eigen::Vector3f& mean, const Eigen::Matrix3f& covariance) {
        Eigen::Matrix3f covarianceInverse = covariance.inverse();
        float detCovariance = covariance.determinant();

        Eigen::Vector3f diff = X - mean;
        float exponent = -0.5 * diff.transpose() * covarianceInverse * diff;

        return (1.0f / std::pow(2 * M_PI, 1.5) * std::sqrt(detCovariance)) * std::exp(exponent);
    }
    */