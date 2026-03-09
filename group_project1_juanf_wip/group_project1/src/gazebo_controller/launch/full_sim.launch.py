import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():

    #bridge_name = LaunchConfiguration('bridge_name')
    #config_file = LaunchConfiguration('config_file')
    #use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #urdf_file_name = 'basic_robot.urdf.xml'
    #urdf = os.path.join(
    #    get_package_share_directory('en613_control'),
    #    urdf_file_name)
    #with open(urdf, 'r') as infp:
    #    robot_desc = infp.read()

    

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
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'),
        Node(
            package='en613_control',
            executable='diffdrive_pid',
            name='diffdrive_pid',
            output='screen'),
    ])