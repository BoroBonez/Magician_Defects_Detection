import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess 

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get the current script's directory
    script_directory = os.path.dirname(os.path.abspath(__file__))

    # Obtain the parent directory of the script directory
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_directory)))))

    urdf = os.path.join(src_dir, 'src/display_with_rviz2/urdf/defects1.urdf')
    rviz = os.path.join(src_dir, 'src/display_with_rviz2/rviz/rviz.rviz')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='optitrack2rviz',
            executable='average_quaternions',
            name='average_quaternions_STL',
            output='screen',
            parameters=[{'input_pose_topic': '/optiTrack/poseSTL'}, {'output_pose_topic': '/average_quaternions/poseSTL'}],
        ),
        Node(
            package='optitrack2rviz',
            executable='average_quaternions',
            name='average_quaternions_camera',
            output='screen',
            parameters=[{'input_pose_topic': '/optiTrack/camera'}, {'output_pose_topic': '/average_quaternions/camera'}],
        ),
        Node(
            package='optitrack2rviz',
            executable='pose_to_tf_STL',
            name='pose_to_tf_STL',
            output='screen',
            parameters=[{'input_pose_topic': '/average_quaternions/poseSTL'}],
        ),
        Node(
            package='optitrack2rviz',
            executable='pose_to_tf_camera',
            name='pose_to_tf_camera',
            output='screen',
            parameters=[{'input_pose_topic': '/average_quaternions/camera'}],
        ),
        Node(
            package='computing_point_cloud',
            executable='stl_point_cloud',
            name='stl_point_cloud',
            output='screen',
        ),
        Node(
            package='computing_point_cloud',
            executable='point_cloud_sub',
            name='transform_pc',
            output='screen',
        ),
        Node(
            package='computing_point_cloud',
            executable='pc_stl_relationship',
            name='pc_stl_relationship',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz]),
    ])