import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    #pkg_ros_gz_sim_demo = get_package_share_directory('ros_gz_sim_demos')

    gz_model_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=PathJoinSubstitution([
        pkg_gazebo_controller,
        'models',
    ])
    )

#    gz_world_path = SetEnvironmentVariable(
#    name='GZ_SIM_RESOURCE_PATH',
#    value=PathJoinSubstitution([
#        pkg_gazebo_controller,
#        'worlds'
#    ])
#    )
    
    sdf_file = os.path.join(pkg_gazebo_controller,'models','gz_robot','model.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_gazebo_controller,
            'worlds',
            'gz_world.sdf'
        ])}.items(),
    )

    gz_topic = '/model/gz_robot'
    joint_state_gz_topic = '/world/car_world' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    cmd_vel_gz_topic = gz_topic + '/cmd_vel'
    #link_tf_gz_topic = gz_topic + '/tf'

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
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', # 1: Good setup
            #cmd_vel_gz_topic + '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        remappings=[
            (joint_state_gz_topic, '/joint_states'),
            #(link_tf_gz_topic, '/tf'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
            #('/cmd_vel', cmd_vel_gz_topic)
            # blank_blank  # 1: Good setup
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
        executable='rviz2'
    )

    diffdrive_controller = Node(
            package='gazebo_controller',
            executable='diffdrive_pid',
            name='diffdrive_pid',
            output='screen')
    
    odom_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'world']
    )

    return LaunchDescription([
        gz_model_path,
        #gz_world_path,
        rviz_launch_arg,
        gazebo,
        bridge,
        robot_state_publisher,
        odom_tf,
        diffdrive_controller,
        rviz,
    ])