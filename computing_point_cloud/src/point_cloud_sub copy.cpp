#include <memory>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;
using std::bind;

class Point_cloud_sub : public rclcpp::Node
{
public:
    Point_cloud_sub()
        : Node("point_cloud_sub")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed2/zed_node/point_cloud/cloud_registered", 10, bind(&Point_cloud_sub::topic_callback, this, _1));
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/optiTrack/camera", rclcpp::SensorDataQoS(), bind(&Point_cloud_sub::poseCallback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/new_point_cloud", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!current_pose_received_)
        {
            RCLCPP_WARN(get_logger(), "Pose not received yet. Skipping point cloud processing.");
            return;
        }

        Eigen::Quaterniond rotation(current_pose_.pose.orientation.w,
                                    current_pose_.pose.orientation.x,
                                    current_pose_.pose.orientation.y,
                                    current_pose_.pose.orientation.z);
        Eigen::Vector3d translation(current_pose_.pose.position.x,
                                   current_pose_.pose.position.y,
                                   current_pose_.pose.position.z);

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translate(translation);
        transform.rotate(rotation);
        std::cout << "Transform Matrix:\n" << transform.matrix() << std::endl;


        Eigen::MatrixXd points(4, msg->width * msg->height);

        for (size_t i = 0; i < msg->width * msg->height; ++i)
        {
            float x = msg->data[j * msg->point_step + msg->fields[0].offset];
            float y = msg->data[j * msg->point_step + msg->fields[1].offset];
            float z = msg->data[j * msg->point_step + msg->fields[2].offset];

            Eigen::Vector4d original_point;
            original_point << msg->data[i * 4], msg->data[i * 4 + 1], msg->data[i * 4 + 2], msg->data[i * 4 + 3];


            Eigen::Vector4d point_to_transform;
            point_to_transform << msg->data[i * 4], msg->data[i * 4 + 1], msg->data[i * 4 + 2], 1.0;
            //std::cout << "point_to_transform:\n" << point_to_transform << std::endl;
            // Transform only the XYZ components
            Eigen::Vector3d transformed_xyz = (transform * point_to_transform).head(3);
            //std::cout << "transformed_xyz:\n" << transformed_xyz << std::endl;

            // Copy the transformed XYZ components and original RGB component
            points.col(i) << transformed_xyz, original_point(3); // Preserve original RGB
            std::cout << "points.col:\n" << points.col(i) << std::endl;
        }

        auto new_point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        
        new_point_cloud_msg->header = msg->header;
        new_point_cloud_msg->width = msg->width;
        new_point_cloud_msg->height = msg->height;
        new_point_cloud_msg->fields = msg->fields;
        new_point_cloud_msg->point_step = msg->point_step;
        new_point_cloud_msg->row_step = msg->row_step;

        // Resize the data array to accommodate the transformed points
        new_point_cloud_msg->data.resize(4 * msg->width * msg->height * sizeof(float));

        // Copy the transformed points into the data array
        for (size_t i = 0; i < msg->width * msg->height; ++i)
        {
            new_point_cloud_msg->data[i * 4] = points(0, i);
            new_point_cloud_msg->data[i * 4 + 1] = points(1, i);
            new_point_cloud_msg->data[i * 4 + 2] = points(2, i);
            new_point_cloud_msg->data[i * 4 + 3] = points(3, i);
        }



        publisher_->publish(*new_point_cloud_msg);
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        current_pose_ = *pose_msg;
        current_pose_received_ = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool current_pose_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Point_cloud_sub>());
    rclcpp::shutdown();
    return 0;
}
