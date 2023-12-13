import sys
import time
import numpy as np
import random
from configparser import ConfigParser

from franka_msgs.msg import FrankaStateCustom
# from franka_msgs.msg import FrankaState
from franka_controllers.msg import PoseWrenchStiff
from geometry_msgs.msg import Pose, PoseStamped, Wrench, Vector3, Quaternion, Point, Twist
# from interactive_markers.menu_handler import *
# from dynamic_reconfigure.server import Server as DynReconfServer
# from franka_controllers.cfg import compliance_paramConfig
# from sensor_msgs.msg import Joy, JointState
# import actionlib
# from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, GraspEpsilon, StopAction, StopGoal
import copy

import rospy
import ros_numpy
from scipy.spatial.transform import Rotation as R
import rosnode
class Franka:

	def __init__(self, config_file='./robot.conf', 
			  home_displacement = (0,0,0), 
			#   low_range=(1,1,0.2), 
			  low_range=(0.1,0.1,0.02), 
			#   high_range=(2,2,1),
			  high_range=(0.2,0.2,0.1),
				 keep_gripper_closed=False, highest_start=False, x_limit=None, y_limit=None, z_limit=None, yaw_limit=None,
				 pitch = 0, roll=180, yaw=0, gripper_action_scale=200, start_at_the_back=False,
				 ):
		###################################
		# ROS stuff
		rospy.init_node('franka_gym_node') 

		# check if rosnode /gazebo exists, if yes, then we are in simulation
		self.in_sim = '/gazebo' in rosnode.get_node_names()
		# self.arm = None
		self.gripper_max_open = 800
		self.gripper_min_open = 0
		# self.zero = (206/100,0/100,120.5/100)	# Units: .1 meters 
		self.zero = (0.206,0.0,0.1205) 
		# home is also relative to zero frame
		self.home = home_displacement
		self.keep_gripper_closed = keep_gripper_closed
		self.highest_start = highest_start
		self.low_range = low_range
		self.high_range = high_range
		self.joint_limits = None
		# self.ip = '192.168.1.246'
		self.gripper_action_scale = gripper_action_scale
		self.start_at_the_back = start_at_the_back

		# Limits
		# self.x_limit = [0.5, 3.5] if x_limit is None else x_limit
		# self.y_limit = [-1.7, 1.3] if y_limit is None else y_limit
		# self.z_limit = [1.4, 3.4] if z_limit is None else z_limit
		# self.yaw_limit = None if yaw_limit is None else yaw_limit
		
		# THESE ARE RELATIVE TO ABOVE DEFINED ZERO FRAME 
		# self.x_limit = [0.05, 0.35] if x_limit is None else x_limit # Leon: this is the original
		self.x_limit = [0.05, 0.5] if x_limit is None else x_limit
		# self.y_limit = [-0.17, 0.13] if y_limit is None else y_limit # Leon: this is the original
		self.y_limit = [-0.5, 0.5] if y_limit is None else y_limit
		self.z_limit = [0.0, 0.4] if z_limit is None else z_limit
		self.yaw_limit = None if yaw_limit is None else yaw_limit 

		# Pitch value - Horizontal or vertical orientation
		self.pitch = pitch
		self.roll = roll
		self.yaw = yaw

	def franka_state_callback(self, msg):
		'''
		Get current franka state and set attributes of marker to current position
		'''
		# self.marker_sp.pose_d = ros_numpy.msgify(Pose, np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
		if not self.initial_pose_found:
			print('initial pose found')
			self.initial_pose.pose = ros_numpy.msgify(Pose, np.transpose(np.reshape(msg.O_T_EE, (4, 4))))

		self.initial_pose_found = True

		self.current_pose.pose = ros_numpy.msgify(Pose, np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
		self.current_pose.header = msg.header

	def test_publisher_callback(self):
		'''
		Just a test publisher moving in a circular motion in the xy plane
		'''
		motion_radius = 0.1
		tangential_vel = 0.1
		angular_vel = tangential_vel/motion_radius
		
		time = rospy.Time.now().to_sec()
		self.sp_xyz[0] = self.initial_pose.pose.position.x + (motion_radius*np.cos(angular_vel*time))
		self.sp_xyz[1] = self.initial_pose.pose.position.y + (motion_radius*np.sin(angular_vel*time))
		self.sp_xyz[2] = self.initial_pose.pose.position.z 

		self.controller_setpoint.pose_d.position = ros_numpy.msgify(Point, self.sp_xyz)

		self.controller_setpoint.header.stamp = rospy.Time.now()

		self.desired_pose_viz_callback()
		self.desired_setpoint_pub.publish(self.controller_setpoint)

	def desired_pose_viz_callback(self):
		'''
		Vizualize the desired pose
		'''

		self.desired_pose_viz.header = self.controller_setpoint.header
		self.desired_pose_viz.pose.position = self.controller_setpoint.pose_d.position
		self.desired_pose_viz.pose.orientation = self.controller_setpoint.pose_d.orientation
		
		self.viz_pose_desired_pub.publish(self.desired_pose_viz)

	def start_robot(self):
		# if self.ip is None:
		# 	raise Exception('IP not provided.')
		# self.arm = XArmAPI(self.ip, is_radian=False)
		# self.arm.motion_enable(enable=False)
		# self.arm.motion_enable(enable=True)
		# if self.arm.error_code != 0:
		# 	self.arm.clean_error()
		# self.set_mode_and_state()
		
		# ###################################
		# # ROS stuff
		# rospy.init_node('franka_gym_node') 

		self.initial_pose_found = False

		self.desired_pose_viz = PoseStamped()
		
		self.home_pose_viz = PoseStamped()
		self.home_pose_viz.header.frame_id = 'panda_link0'

		self.zero_pose_viz = PoseStamped()
		self.zero_pose_viz.header.frame_id = 'panda_link0'

		# self.marker_sp = PoseWrenchStiff()
		self.initial_pose = PoseStamped()
		self.current_pose = PoseStamped()

		# self.controller_setpoint = PoseStamped()
		self.controller_setpoint = PoseWrenchStiff()
		self.controller_setpoint.header.frame_id = 'panda_link0'
		self.controller_setpoint.wrench_d = Wrench(Vector3(0., 0., 0.), Vector3(0., 0., 0.)) 
		if self.in_sim:
			# self.translation_stiffness = np.array([500., 500., 500.])
			# self.rotation_stiffness = 1.*self.translation_stiffness
			# copied over from the contact generation policy
			self.translation_stiffness = np.array([200., 200., 200.])
			self.rotation_stiffness = np.array([100., 100., 100.])
			self.controller_setpoint.tau_filter_coeff = 1.0 # NEED TO SET THIS NONZERO! Otherwise controller does nothing
		else:
			# self.marker_sp.cartesian_stiffness = (500., 500., 500., 1500., 1200., 1500.) # for stiff tracking, 200 for trans 10 for orientation...
			# self.translation_stiffness = np.array([400., 400., 400.])
			self.translation_stiffness = np.array([900., 900., 900.]) # this is very stiff tracking, still has some errors though. 
			# self.rotation_stiffness = 1.3*self.translation_stiffness
			self.rotation_stiffness = 2.0*self.translation_stiffness
			# self.marker_sp.tau_filter_coeff = 0.06
			self.controller_setpoint.tau_filter_coeff = 1.0
	
		self.translation_damping = 2.*np.sqrt(self.translation_stiffness)
		self.rotation_damping = 2.*np.sqrt(self.rotation_stiffness)

		if not self.in_sim:
			# reduce the rotation damping 
			self.rotation_damping *= 0.0	
			# reduce the translation damping
			# make this 50.0 for translation damping
			self.translation_damping *= 1.0

		self.controller_setpoint.cartesian_stiffness = tuple(np.concatenate((self.translation_stiffness, self.rotation_stiffness)))
		self.controller_setpoint.cartesian_damping = tuple(np.concatenate((self.translation_damping, self.rotation_damping)))
		
		# self.position_limits = np.array([[0.2, 0.8], [-0.6, 0.6], [-0.1, 0.5]]) FROM MY TELEOP CODE
		# self.xy_max_r = 0.75 FROM MY TELEOP CODE
	
		# if self.use_gripper:
        #     # self.keyboard_teleop_sub = rospy.Subscriber("/panda/cmd_vel", Twist, self.keyboard_teleop_callback)
        #     # use a timer based callback to read keyboard input
        #     self.keyboard_teleop_timer = rospy.Timer(rospy.Duration(0.01), self.keyboard_teleop_callback)
        #     self.gripper_state_sub = rospy.Subscriber('/panda/franka_gripper/joint_states', JointState, self.gripper_state_cb)                            
        #     self.gripper_positions = None
        #     self.gripper_width = None

        #     self.gripper_move_act = actionlib.SimpleActionClient('/panda/franka_gripper/move', MoveAction)
        #     self.gripper_grasp_act = actionlib.SimpleActionClient('/panda/franka_gripper/grasp', GraspAction)
        #     self.gripper_stop_act = actionlib.SimpleActionClient('/panda/franka_gripper/stop', StopAction)

        #     rospy.loginfo('waiting for gripper action services...')
        #     self.gripper_move_act.wait_for_server()
        #     self.gripper_grasp_act.wait_for_server()
        #     self.gripper_stop_act.wait_for_server()
        #     rospy.loginfo('found gripper action services!')

        #     self.gripper_open_goal = MoveGoal(width=0.08, speed=0.1)
        #     self.gripper_grasp_goal = GraspGoal(width=0.001, speed=0.07, force=50., epsilon = GraspEpsilon(inner=.1, outer=.1))
        #     self.gripper_stop_goal = StopGoal()
	
		# create a subscriber for the franka state
		self.state_sub = rospy.Subscriber("/panda/franka_state_controller_custom/franka_states", FrankaStateCustom, self.franka_state_callback)
		
		# Get initial pose for the interactive marker
		while not self.initial_pose_found:
			rospy.sleep(1)

		self.controller_setpoint.pose_d = copy.deepcopy(self.initial_pose.pose)
		# Publish visualization of where franka is moving
		self.viz_pose_desired_pub = rospy.Publisher("/panda/hybrid_impedance_wrench_controller/pose_viz", PoseStamped, queue_size=1, tcp_nodelay=True)

		# set up publisher for both home and zero poses
		self.viz_home_pose_pub = rospy.Publisher("/panda/home_pose", PoseStamped, queue_size=1, tcp_nodelay=True)
		self.viz_zero_pose_pub = rospy.Publisher("/panda/zero_pose", PoseStamped, queue_size=1, tcp_nodelay=True)

		# setup a timer based callback to publish the home and zero pose at 1 Hz
		self.home_zero_pose_timer = rospy.Timer(rospy.Duration(1.0), self.home_zero_pose_callback)

		# create a publisher for the franka command
		self.desired_setpoint_pub = rospy.Publisher("/panda/hybrid_impedance_wrench_controller/pose_wrench_desired", PoseWrenchStiff, queue_size=1)
		# self.desired_setpoint_pub = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=1)
	
		###################################
	def home_zero_pose_callback(self, event):
		'''
		Publish home and zero pose for visualization
		'''
		home_pos = np.array([self.zero[0] + self.home[0], self.zero[1] + self.home[1], self.zero[2] + self.home[2], self.roll, self.pitch, self.yaw])
		self.home_pose_viz.pose = self.sixd_pos_to_pose(home_pos)
		# print('home pose: ', home_pose)

		zero_pos = np.array([self.zero[0], self.zero[1], self.zero[2], self.roll, self.pitch, self.yaw])
		self.zero_pose_viz.pose = self.sixd_pos_to_pose(zero_pos)
		# print('zero pose: ', zero_pose)
		# self.home_pose_viz.pose = ros_numpy.msgify(Pose, np.array([self.zero[0] + self.home[0], 
		# 													 self.zero[1] + self.home[1], 
		# 													 self.zero[2] + self.home[2], 
		# 													 self.roll, self.pitch, self.yaw])
		# 													)
		self.viz_home_pose_pub.publish(self.home_pose_viz)

		# self.zero_pose_viz.pose = ros_numpy.msgify(Pose, np.array([self.zero[0], self.zero[1], self.zero[2], self.roll, self.pitch, self.yaw]))
		self.viz_zero_pose_pub.publish(self.zero_pose_viz)

	def sixd_pos_to_pose(self, pos):
		'''
		Convert a 6dof position to a Pose
		'''
		pose = Pose()
		pose.position.x = pos[0]
		pose.position.y = pos[1]
		pose.position.z = pos[2]
		pose.orientation = ros_numpy.msgify(Quaternion, R.from_euler('xyz', [pos[3], pos[4], pos[5]], degrees=True).as_quat())
		return pose
	
	def set_mode_and_state(self, mode=0, state=0):
		# self.arm.set_mode(mode)
		# self.arm.set_state(state=state)
		raise NotImplementedError

	def clear_errors(self):
		# self.arm.clean_warn()
		# self.arm.clean_error()
		raise NotImplementedError

	def has_error(self):
		# return self.arm.has_err_warn
		raise NotImplementedError
	
	def reset(self, home = False, reset_at_home=True):
		# if self.arm.has_err_warn:
		# 	self.clear_errors()

		if home:
			if reset_at_home:
				self.move_to_home()
			else:
				self.move_to_zero()

		# 	if self.keep_gripper_closed:
		# 		self.close_gripper_fully()
		# 	else:
		# 		self.open_gripper_fully()

	def move_to_home(self, open_gripper=False):
		'''
		retain current orientation but move to home position

		ON REAL MAY REQUIRE SOME MOTION PLANNING TO AVOID HITTING STUFF
		'''
		pos = self.get_position()
		pos[0] = self.home[0]
		pos[1] = self.home[1]
		pos[2] = self.home[2]
		self.set_position(pos)
		
		# if open_gripper and not self.keep_gripper_closed:
		# 	self.open_gripper_fully()
	
	def set_random_pos(self):
		'''
		retain current orientation but move to random position
		'''
		# self.clear_errors()
		# self.set_mode_and_state()
		pos = self.get_position()
		
		# Move up
		pos[2] = self.z_limit[1]
		self.set_position(pos)

		# Set random pos
		x_disp = self.low_range[0] + np.random.rand()*(self.high_range[0] - self.low_range[0])
		y_disp = self.low_range[1] + np.random.rand()*(self.high_range[1] - self.low_range[1])
		z_disp = self.low_range[2] + np.random.rand()*(self.high_range[2] - self.low_range[2])
		
		pos[0] = self.home[0] + x_disp * np.random.choice([-1,1])		# Here we sample in a square ring around the home 
		pos[1] = self.home[1] + y_disp * np.random.choice([-1,1])		# Here we sample in a square ring around the home 
		pos[2] = self.home[2] + z_disp if not self.highest_start else self.z_limit[1] 									# For z we jsut sample from [a,b]
		self.set_position(pos)
		
		# if self.keep_gripper_closed:
		# 	self.close_gripper_fully()
		# else:
		# 	self.open_gripper_fully()

	def move_to_zero(self):
		'''
		retain current orientation but move to the user defined zero position
		'''
		pos = self.get_position()
		# set 0 or the limit if 0 is outside
		pos[0] = min(max(self.x_limit[0],0), self.x_limit[1])# 0
		pos[1] = min(max(self.y_limit[0],0), self.y_limit[1])# 0
		# set at upper z limit if highest start is true
		pos[2] = min(max(self.z_limit[0],0), self.z_limit[1]) if not self.highest_start else self.z_limit[1] # 0

		self.set_position(pos)

	def set_position(self, pos, wait=False, use_roll=False, use_pitch=False, use_yaw=False):
		pos = self.limit_pos(pos)
		x = (pos[0] + self.zero[0])
		y = (pos[1] + self.zero[1])
		z = (pos[2] + self.zero[2])
		roll = pos[3] if use_roll else self.roll
		pitch = pos[4] if use_pitch else self.pitch
		yaw = pos[5] if use_yaw else self.yaw
		# self.arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, wait=wait)

		# convert to Pose and publish
		self.controller_setpoint.pose_d.position = ros_numpy.msgify(Point, np.array([x,y,z]))
		self.controller_setpoint.pose_d.orientation = ros_numpy.msgify(Quaternion, R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat())
		self.controller_setpoint.header.stamp = rospy.Time.now()

		self.desired_pose_viz_callback()
		self.desired_setpoint_pub.publish(self.controller_setpoint)

	def get_position(self):
		pos = ros_numpy.numpify(self.current_pose.pose.position)
		rpy = R.from_quat(ros_numpy.numpify(self.current_pose.pose.orientation)).as_euler('xyz', degrees=True)
		x = (pos[0] - self.zero[0])
		y = (pos[1] - self.zero[1])
		z = (pos[2] - self.zero[2])
		return np.array([x,y,z, rpy[0], rpy[1], rpy[2]]).astype(np.float32)

	def get_gripper_position(self):
		# code, pos = self.arm.get_gripper_position()
		# if code!=0:
		# 	raise Exception('Correct gripper angle cannot be obtained.')
		# return pos
		raise NotImplementedError

	def open_gripper_fully(self):
		# self.set_gripper_position(self.gripper_max_open)
		raise NotImplementedError


	def close_gripper_fully(self):
		# self.set_gripper_position(self.gripper_min_open)
		raise NotImplementedError
		

	def open_gripper(self):
		# self.set_gripper_position(self.get_gripper_position() + self.gripper_action_scale)
		raise NotImplementedError

	def close_gripper(self):
		# self.set_gripper_position(self.get_gripper_position() - self.gripper_action_scale)
		raise NotImplementedError

	def set_gripper_position(self, pos, wait=False):
		'''
		wait: To wait till completion of action or not
		'''
		# if pos<self.gripper_min_open:
		# 	pos = self.gripper_min_open
		# if pos>self.gripper_max_open:
		# 	pos = self.gripper_max_open
		# self.arm.set_gripper_position(pos, wait=wait, auto_enable=True)
		raise NotImplementedError

	def get_servo_angle(self):
		# code, angles = self.arm.get_servo_angle()
		# if code!=0:
			# raise Exception('Correct servo angles cannot be obtained.')
		# return angles
		raise NotImplementedError


	def set_servo_angle(self, angles, is_radian=None):
		'''
		angles: List of length 8
		'''
		# self.arm.set_servo_angle(angle=angles, is_radian=is_radian)
		raise NotImplementedError
	
	def limit_pos(self, pos):
		pos[0] = max(self.x_limit[0], pos[0])
		pos[0] = min(self.x_limit[1], pos[0])
		pos[1] = max(self.y_limit[0], pos[1])
		pos[1] = min(self.y_limit[1], pos[1])
		pos[2] = max(self.z_limit[0], pos[2])
		pos[2] = min(self.z_limit[1], pos[2])
		if self.yaw_limit is not None:
			pos[5] = max(self.yaw_limit[0], pos[5])
			pos[5] = min(self.yaw_limit[1], pos[5])
		return pos

if __name__ == '__main__':
	arm = Franka()
	arm.start_robot()
	arm.move_to_zero()
	time.sleep(5)

	# sleep
	arm.set_random_pos()
	time.sleep(5)
