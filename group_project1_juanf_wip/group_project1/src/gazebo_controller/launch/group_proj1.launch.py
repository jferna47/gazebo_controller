import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    pkg_gazebo_controller = get_package_share_directory('gazebo_controller')
    #pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    #pkg_ros_gz_sim_demo = get_package_share_directory('ros_gz_sim_demos')

    sdf_file = os.path.join(pkg_gazebo_controller,'models','gz_robot.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_controller, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_gazebo_controller,
            'models',
            'gz_robot.sdf'
        ])}.items(),
    )

    gz_topic = '/model/'
    joint_state_gz_topic = '/world/car_world' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_gazebo_controller)]
    )

    return LaunchDescription([
        rviz_launch_arg,
        gazebo,
        bridge,
        robot_state_publisher,
        rviz
    ])