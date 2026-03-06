import math
import tf2_ros
import rclpy
from time import sleep
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, TransformStamped # Enable use of the geometry_msgs/Twist message type
from tf2_ros import TransformBroadcaster, TransformException, LookupException
from example_interfaces.srv import Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
import numpy as np
#from .diffdrive_robot import DiffDriveRobot

class DiffDrivePID(Node):
    
    def __init__(self):
        
        super().__init__('diffdrive_pid')

        # DiffDriveRobot object
		# we are setting dimension properties here (in meters)
        length = 2 # meters
        wheel_sep = 1.2 # meters
        wheel_radius = 0.4 # meters

        self.vehicle = DiffDriveRobot(length,wheel_sep,wheel_radius)

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.turn_rate = 0.0

        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.goal_pose_qx = 0.0
        self.goal_pose_qy = 0.0
        self.goal_pose_qz = 0.0
        self.goal_pose_qw = 0.0

        
        self.kp = 0.1
        #self.ki = 0 # No integral portion for this
        self.kd = 0.05

        self.dt = 1.0/30.0
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
      
        # subscriber TF2 (transform listener)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        self.to_frame = 'chassis'
        self.from_frame = 'odom'

        # subscriber /goal_pose
        self.goal_pose_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_pose_update,10)
        self.goal_pose_sub  # prevent unused variable warning
        
        # publisher --> /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Telling us that node has been initialized and started
        self.node_name = self.get_name()
        self.get_logger().info("{0} started".format(self.node_name))


    def timer_callback(self):

        try:
            when = rclpy.time.Time()
            #odom_chassis = self.tf_buffer.lookup_transform(self.to_frame, self.from_frame, when, timeout=Duration(seconds=5.0))
            odom_chassis = self.tf_buffer.lookup_transform(self.from_frame, self.to_frame, when, timeout=Duration(seconds=5.0))


        except tf2_ros.LookupException:
            self.get_logger().info('Transform isn\'t available, waiting...')
            sleep(1)
            return

        print(f'X position of robot = {odom_chassis.transform.translation.x}')
        print(f'Y position of robot = {odom_chassis.transform.translation.y}')
        #print(f'Z position of robot = {odom_chassis.transform.translation.z}') # Doesn't really matter

        old_x_pos = self.x_pos
        old_y_pos = self.y_pos
        old_qx    = self.qx
        old_qy    = self.qy
        old_qz    = self.qz
        old_qw    = self.qw
        old_theta = self.theta

        self.x_pos = odom_chassis.transform.translation.x
        self.y_pos = odom_chassis.transform.translation.y

        self.qx = odom_chassis.transform.rotation.x 
        self.qy = odom_chassis.transform.rotation.y 
        self.qz = odom_chassis.transform.rotation.z 
        self.qw = odom_chassis.transform.rotation.w
        temp_qtr = np.array([self.qw, self.qx, self.qy, self.qz])
        temp_angles = quaternion_to_euler(temp_qtr)
        self.theta = temp_angles[2] # Getting theta aka the yaw

        self.x_vel = (self.x_pos - old_x_pos)/self.dt
        self.y_vel = (self.y_pos - old_y_pos)/self.dt

        delta_theta = (((self.theta - old_theta) + np.pi) % (2*np.pi)) - np.pi
        self.turn_rate = (delta_theta)/self.dt

        x_p = self.x_pos + self.vehicle.W * np.cos(self.theta)
        y_p = self.y_pos + self.vehicle.W * np.sin(self.theta)
        
        rot_mat  = np.array([[np.cos(self.theta), np.sin(self.theta)],
                            [-np.sin(self.theta), np.cos(self.theta)]])
    
        prop_mat = np.array([[self.goal_pose_x - x_p],
                            [self.goal_pose_y - y_p]])
    
        deriv_mat = np.array([[self.x_vel],
                                [self.y_vel]])
    
        pd_mat = (self.kp * prop_mat) - (self.kd * deriv_mat)

        control_vel = np.matmul(rot_mat, pd_mat)

        control_vel = control_vel.flatten()
        
        cmd_forward_vel = float(control_vel[0])

        # To stop the PD from commanding negative forward velocity
        #if cmd_forward_vel < 0:
        #    cmd_forward_vel = 0.0

        cmd_turn_rate = float(control_vel[1] / self.vehicle.W)

        robot_vel_cmd = Twist()
        robot_vel_cmd.linear.x = cmd_forward_vel
        robot_vel_cmd.linear.y = 0.0
        robot_vel_cmd.angular.z = cmd_turn_rate # Store the wheel rotation rate
        
        print(f'Commanded forward velocity of robot = {robot_vel_cmd.linear.x}')
        print(f'Commanded turn rate of robot = {robot_vel_cmd.angular.z}')

        self.cmd_vel_pub.publish(robot_vel_cmd) # Publish the forward speed and yaw rate to the topic 
        #print('sent velocity commands')

    def goal_pose_update(self,new_goal):
      
        self.goal_pose_x = new_goal.pose.position.x
        self.goal_pose_y = new_goal.pose.position.y

        qx = new_goal.pose.orientation.x
        qy = new_goal.pose.orientation.y
        qz = new_goal.pose.orientation.z
        qw = new_goal.pose.orientation.w

        self.goal_pose_q = np.array([qw, qx, qy, qz])

        angles = quaternion_to_euler(self.goal_pose_q)

        self.goal_pose_theta = angles[2] # We only care about angle with respect to zaxis (yaw)
            
class DiffDriveRobot:

	def __init__(self, length, wheel_sep, wheel_radius):
		
		self.L = length
		self.W = wheel_sep # This is needed for correct kinematics
		self.R = wheel_radius


	def forward(self, state, wheel_rates):

		x0 = state[0]
		y0 = state[1]
		theta0 = state[2]

		lw_rate = wheel_rates[0]
		rw_rate = wheel_rates[1]

		# 
		forward_vel = (self.R / 2)*(lw_rate + rw_rate)
		#normal_vel = 0
		turn_rate = (self.R / (self.W ))*(rw_rate - lw_rate)
		v = np.array([forward_vel, 0.0, turn_rate])

		return v

	
	def inverse(self, state, velocity):

		x0 = state[0]
		y0 = state[1]
		theta0 = state[2]

		forward_vel = velocity[0]
		#normal_vel = 0.0 velocity[1]
		turn_rate = velocity[2]

		# 
		lw_rate = forward_vel / self.R - (turn_rate * (self.W ))/(2 * self.R)
		rw_rate = forward_vel / self.R + (turn_rate * (self.W ))/(2 * self.R)

		u = np.array([lw_rate, rw_rate])

		return u

def quaternion_to_euler(Q: np.ndarray) -> np.ndarray:

    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    r11 = 1 - 2 * (q2**2 + q3**2)
    r21 = 2 * (q1*q2 + q0*q3)
    r31 = 2 * (q1*q3 - q0*q2)
    r32 = 2 * (q2*q3 + q0*q1)
    r33 = 1 - 2 * (q1**2 + q2**2)

    alpha = np.arctan2(r32, r33)
    beta = np.arctan2(-r31, np.sqrt(r32**2 + r33**2))
    gamma = np.arctan2(r21, r11)

    eulerMat = np.array([alpha, beta, gamma])

    return eulerMat

def main(args=None):
    rclpy.init(args=args)

    diffdrive_pid = DiffDrivePID()

    rclpy.spin(diffdrive_pid)

    diffdrive_pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


