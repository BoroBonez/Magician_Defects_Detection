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

    socat_command = "socat UDP4-RECV:1511,bind=239.255.42.99,ip-add-membership=239.255.42.99:enp3s0,reuseaddr - | hexdump"

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        ExecuteProcess(
            #cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py', 'camera_model:=zed2'],
            cmd = ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',  'depth_module.profile:=1280x720x30', 'pointcloud.enable:=true'],
            output='screen',),
        ExecuteProcess(
            cmd=['bash', '-c', f"{socat_command} > /dev/null 2>&1"],
            output='screen',),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='optitrack_interface',
            executable='optitrack',
            name='optitrack_node_STL',
            output='screen',
            parameters=[{'robot_id': 47}, {'topic': '/optiTrack/poseSTL'}]),
        Node(
            package='optitrack_interface',
            executable='optitrack',
            name='optitrack_node_camera',
            output='screen',
            parameters=[{'robot_id': 48}, {'topic': '/optiTrack/camera'}]),
        Node(
            package='optitrack2rviz',
            executable='pose_to_tf_STL',
            name='pose_to_tf_STL',
            output='screen',
        ),
        Node(
            package='optitrack2rviz',
            executable='pose_to_tf_camera',
            name='pose_to_tf_camera',
            output='screen',
        ),
        Node(
            package='computing_point_cloud',
            executable='point_cloud_sub',
            name='transform_pc',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz]),
    ])