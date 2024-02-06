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
#include <Eigen/Sparse>
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
#include <Eigen/SparseLU>

#include "defines.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using std::bind;
using std::placeholders::_1;

class Mesh {
    public:
        Mesh(std::size_t size, Eigen::MatrixXd V_copy, Eigen::MatrixXi F_copy) : n_of_triangles(size), V(V_copy), F(F_copy) {

            normal_to_each_face.resize(n_of_triangles);
            x.resize(n_of_triangles);
            P.resize(n_of_triangles, n_of_triangles);

            //initialization of the normal direction of each face
            std::vector<int> connectedVertices_id;
            for (std::size_t  i = 0; i < (std::size_t )n_of_triangles; ++i) {
                x[i] = 0;
                P.insert(i, i) = 0.05 * 0.05;

                Eigen::Vector3d vertex1(V.row(F.row(i)[0])), vertex2(V.row(F.row(i)[1])), vertex3(V.row(F.row(i)[2]));
                Eigen::Vector3d edge1(vertex1[0] - vertex2[0], vertex1[1] - vertex2[1], vertex1[2] - vertex2[2]);
                Eigen::Vector3d edge2(vertex1[0] - vertex3[0], vertex1[1] - vertex3[1], vertex1[2] - vertex3[2]);

                Eigen::Vector3d normal;
                normal = edge1.cross(edge2);
                normal = normal / normal.norm();
                normal_to_each_face[i] = normal;
            }

        }

        float h_x(std::size_t index){
            return x[index];
        }

        double lowest_state(){
            float min = std::numeric_limits<double>::max();
            for (std::size_t i = 0; i < n_of_triangles; ++i) {
                    if (std::abs(x[i]) < min){
                        min = std::abs(x[i]);
                    }
                }
            return min;
        }

        double highest_state(){
            float max = std::numeric_limits<double>::min();
            for (std::size_t i = 0; i < n_of_triangles; ++i) {
                    if (std::abs(x[i]) > max){
                        max = std::abs(x[i]);
                    }
                }
            return max;
        }

        void print_state(){
            std::cout << "\n" << std::endl;
            for (std::size_t i = 0; i < n_of_triangles; ++i) {
                std::cout << "the state x[" << i << "] is: " << x[i] << "\n" << std::endl;
            }
        }

        void generate_state_point_cloud(){
            // Initialize X_state
            X_state = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
            X_state->points.reserve(n_of_triangles);

            for (std::size_t  i = 0; i < (std::size_t )n_of_triangles; ++i) {
                Eigen::Vector3d vertex1(V.row(F.row(i)[0])), vertex2(V.row(F.row(i)[1])), vertex3(V.row(F.row(i)[2]));

                Eigen::Vector3d displacement_vector( normal_to_each_face[i]*x[i] );

                pcl::PointXYZRGB p;
                p.x = (vertex1[0] + vertex2[0] + vertex3[0]) / 3 + displacement_vector[0];
                p.y = (vertex1[1] + vertex2[1] + vertex3[1]) / 3 + displacement_vector[1];
                p.z = (vertex1[2] + vertex2[2] + vertex3[2]) / 3 + displacement_vector[2];
                X_state->points.push_back(p);
            }

        }

        std::size_t n_of_triangles;
        std::vector<Eigen::Vector3d> normal_to_each_face;
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        Eigen::VectorXd x;
        Eigen::SparseMatrix<double>  P;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr X_state;
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

        publisher_state = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/X", 10
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
        //if (msg->header.stamp.sec < STL_pose->header.stamp.sec ||
        //    (msg->header.stamp.sec == STL_pose->header.stamp.sec &&
        //     msg->header.stamp.nanosec < STL_pose->header.stamp.nanosec)) {
        //    RCLCPP_WARN(
        //            get_logger(),
        //            "camera point cloud has a lower stamp than STL point cloud"
        //    );
        //    return;
        //}
        if (!current_camera_pose) {
            RCLCPP_WARN(
                    get_logger(),
                    "camera pose not received yet. Skipping point cloud processing."
            );
            return;
        }
        //if (msg->header.stamp.sec < camera_pose->header.stamp.sec ||
        //    (msg->header.stamp.sec == camera_pose->header.stamp.sec &&
        //     msg->header.stamp.nanosec < camera_pose->header.stamp.nanosec)) {
        //    RCLCPP_WARN(
        //            get_logger(),
        //            "camera point cloud has a lower stamp than camera pose"
        //    );
        //    return;
        //}

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
        float step = -0.05;

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
        std::vector<double> dynamicVector_m;
        std::vector<Eigen::RowVectorXd> H_rows;
        std::vector<double> R_diagonal;

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
                std::cout << "there are still " << cloudPC->points.size() - i << " to analyze\n" << std::endl;
                pcl::PointXYZRGB p;
                p.x = origin.x() + hits.front().t * direction.x();
                p.y = origin.y() + hits.front().t * direction.y();
                p.z = origin.z() + hits.front().t * direction.z();

                //Bayesian filter matrices filling
                dynamicVector_m.push_back( vector_from_p_to_p(p, cloudPC->points[i]).dot(mesh_ptr->normal_to_each_face[hits.front().id]) );
                Eigen::RowVectorXd H_row = Eigen::RowVectorXd::Zero(mesh_ptr->n_of_triangles);
                H_row[hits.front().id] = 1;
                H_rows.push_back(H_row);
                R_diagonal.push_back( R_i(cloudPC->points[i]) );

                //mesh_ptr->triangles[hits.front().id]->normal_dist_camera = 
                augmentedcloud->points.push_back(p);
                id_point_at_index.push_back(hits.front().id);
                count[hits.front().id]++;
            } 

        }
        // Matrix generation
        std::size_t n_hits = dynamicVector_m.size();

        // Convert dynamicVector_m to a Eigen::VectorXd
        Eigen::VectorXd m(dynamicVector_m.size());
        m = Eigen::Map<Eigen::VectorXd>(dynamicVector_m.data(), n_hits);

        // Convert H_rows to a sparse matrix
        Eigen::SparseMatrix<double> H(n_hits, mesh_ptr->n_of_triangles);
        for (size_t i = 0; i < n_hits; ++i) {
            for (size_t j = 0; j < mesh_ptr->n_of_triangles; ++j) {
                if (H_rows[i][j] != 0) {
                    H.insert(i, j) = H_rows[i][j];
                }
            }
        }

        // Convert R_diagonal to a sparse matrix
        Eigen::SparseMatrix<double> R(n_hits, n_hits);
        for (size_t i = 0; i < n_hits; ++i) {
            R.insert(i, i) = R_diagonal[i];
        }
        std::cout << "\nThe size of H is: (" << H.rows() << ", " << H.cols() << ")\n" << std::endl;
        std::cout << "\nThe size of P is: (" << mesh_ptr->P.rows() << ", " << mesh_ptr->P.cols() << ")\n" << std::endl;
        std::cout << "\nThe size of R is: (" << R.rows() << ", " << R.cols() << ")\n" << std::endl;
        H.makeCompressed();
        R.makeCompressed();


        //Bayesian filter matrices computing

        //performing S = H * mesh_ptr->P * H.transpose() + R;
        Eigen::SparseMatrix<double> middle_step = (mesh_ptr->P * H.transpose()).pruned();
        middle_step.makeCompressed();
        Eigen::SparseMatrix<double> S = H * middle_step;
        S += R;
        S.makeCompressed();
        S = S.pruned();
        std::cout << "\nThe size of S is: (" << S.rows() << ", " << S.cols() << ")\n" << std::endl;

        //perform W = mesh_ptr->P * H.transpose() * S_inverse
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.compute(S);
        Eigen::SparseMatrix<double> I(n_hits,n_hits);
        I.setIdentity();
        Eigen::SparseMatrix<double> S_inverse = (solver.solve(I));
        S_inverse.makeCompressed();
        std::cout << "\nThe size of S_inverse is: (" << S_inverse.rows() << ", " << S_inverse.cols() << ")\n" << std::endl;

        Eigen::SparseMatrix<double> W =  middle_step * S_inverse;
        std::cout << "i did matrix W\n" << std::endl;

        //perform mesh_ptr->x = mesh_ptr->x + W * (m - H * mesh_ptr->x);
        m -= H * mesh_ptr->x;
        mesh_ptr->x += W * m;

        //mesh_ptr->P = (sparseIdentity - W * H) * mesh_ptr->P;
        Eigen::SparseMatrix<double> sparseIdentity(mesh_ptr->n_of_triangles, mesh_ptr->n_of_triangles);
        sparseIdentity.setIdentity();

        //set the new color to show the heatmap
        for (std::size_t i = 0; i < augmentedcloud->points.size(); ++i) {

                int id = id_point_at_index[i];
                double threshold = 0.01;

                if (std::abs(mesh_ptr->x[id]) > threshold) {
                        // yellow 255, 255, 0
                        // from yellow to red
                        augmentedcloud->points[i].r = 255;
                        augmentedcloud->points[i].g = ( 1 - (std::abs(mesh_ptr->x[id]) - threshold)/(mesh_ptr->highest_state() - threshold) ) * 255;
                        augmentedcloud->points[i].b = 0;

                } else {
                        // from yellow to green
                        augmentedcloud->points[i].r = ( 1 - (std::abs(mesh_ptr->x[id])  - threshold)/(mesh_ptr->lowest_state()  - threshold) ) * 255;
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

        mesh_ptr->generate_state_point_cloud();

        pcl::toPCLPointCloud2(*(mesh_ptr->X_state), pcl_pc2);
        auto new_point_cloud_msg2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::fromPCL(pcl_pc2, *new_point_cloud_msg2);
        new_point_cloud_msg2->header = msg->header;
        publisher_state->publish(*new_point_cloud_msg2);

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

    Eigen::Vector3d vector_from_p_to_p(pcl::PointXYZRGB point1, pcl::PointXYZRGB point2){
        Eigen::Vector3d diff_vector; 
        diff_vector << point2.x - point1.x,
                       point2.y - point1.y,
                       point2.z - point1.z;

        return diff_vector;
    }

    float R_i(pcl::PointXYZRGB S){
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

        return std * std;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_camera_PC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_, publisher_state;
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