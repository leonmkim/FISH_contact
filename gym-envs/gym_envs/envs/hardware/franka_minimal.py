#!/usr/bin/env python3

import sys
import time
import numpy as np
import random
from configparser import ConfigParser

# from franka_msgs.msg import FrankaStateCustom
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Pose, PoseStamped, Wrench, Vector3, Quaternion, Point, Twist
# from franka_controllers.msg import PoseWrenchStiff
# from interactive_markers.menu_handler import *
# from dynamic_reconfigure.server import Server as DynReconfServer
# from franka_controllers.cfg import compliance_paramConfig
# from sensor_msgs.msg import Joy, JointState
# from scipy.spatial.transform import Rotation as R
# import actionlib
# from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal, GraspEpsilon, StopAction, StopGoal

import rospy
import ros_numpy

from scipy.spatial.transform import Rotation as R

class Franka:

	def __init__(self,):
		# ROS stuff
		###################################

		# self.initial_pose_found = False

		# self.pose_viz = PoseStamped()

		# self.marker_sp = PoseWrenchStiff()
		# self.marker_sp.header.frame_id = 'panda_link0'
		# self.marker_sp.wrench_d = Wrench(Vector3(0., 0., 0.), Vector3(0., 0., 0.)) 
		# self.translation_stiffness = np.array([400., 400., 400.])
		# # self.rotation_stiffness = 1.3*self.translation_stiffness
		# self.rotation_stiffness = 1.6*self.translation_stiffness
		# # self.marker_sp.tau_filter_coeff = 0.06
		# self.marker_sp.tau_filter_coeff = 1.0
		# self.translation_damping = 2.*np.sqrt(self.translation_stiffness)
		# self.rotation_damping = 2.*np.sqrt(self.rotation_stiffness)
        # # reduce the rotation damping 
		# self.rotation_damping *= 0.0
        # # reduce the translation damping
        # # make this 50.0 for translation damping
		# self.translation_damping *= 1.0
		# self.marker_sp.cartesian_stiffness = tuple(np.concatenate((self.translation_stiffness, self.rotation_stiffness)))
		# self.marker_sp.cartesian_damping = tuple(np.concatenate((self.translation_damping, self.rotation_damping)))
		
		# self.linear_vels = np.array([0., 0., 0.])
		# self.angular_vels = np.array([0., 0., 0.])
		# self.sp_xyz = np.array([0.,0.,0.])
		# self.xy_max_r = 0.75
	
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
		# self.state_sub = rospy.Subscriber("/panda/franka_state_controller_custom/franka_states", FrankaStateCustom, self.franka_state_callback)
		self.state_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
		# self.state_sub = rospy.Subscriber("/panda/hybrid_impedance_wrench_controller/pose_wrench_desired", PoseWrenchStiff, self.franka_state_callback)
		
		# self.state_sub = rospy.Subscriber("/cmd_vel", Twist, self.franka_state_callback)
		print('made callback')
		
		rospy.init_node('franka_node') 
		print('init node')

		# make sure sub is connected
		print(self.state_sub.get_num_connections())
		while self.state_sub.get_num_connections() == 0:
			rospy.sleep(1)
		print('connected')

		# # Get initial pose for the interactive marker
		# while not self.initial_pose_found:
		# 	rospy.sleep(1)

		# self.O_T_EE = PoseStamped()

		# # Publish visualization of where franka is moving
		# self.pose_desired_pub = rospy.Publisher("/hybrid_impedance_wrench_controller/pose_viz", PoseStamped, queue_size=1, tcp_nodelay=True)
		self.hz = 100.0
		# # run desired pose wrench stiffness publisher
		self.sp_timer = rospy.Timer(rospy.Duration(1/self.hz), lambda msg: self.publisher_callback(msg))
		# # create a publisher for the franka command
		# self.pose_wrench_pub = rospy.Publisher(
        #     "/panda/hybrid_impedance_wrench_controller/pose_wrench_desired", PoseWrenchStiff, queue_size=1)

		# rospy.spin()
	def franka_state_callback(self, msg):
		'''
		Get current franka state and set attributes of marker to current position
		'''
		# print('in franka_state_callback')
		# print(msg)
		# self.marker_sp.pose_d = ros_numpy.msgify(Pose, np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
		state = np.transpose(np.reshape(msg.O_T_EE, (4, 4)))

		# self.robot_pose = ros_numpy.numpify(self.marker_sp.pose_d)
		print(state)

		# self.initial_pose_found = True
	
	def publisher_callback(self, msg):
		# '''
		# Publish desired pose after calculating new location with velocity and bounds
		# '''
		# # marker_sp.pose_d.position.x = max(min(marker_sp.pose_d.position.x + linear_vels[0], position_limits[0][1]), position_limits[0][0])

		# self.sp_xyz[0] = max(self.marker_sp.pose_d.position.x + self.linear_vels[0], self.position_limits[0, 0])
		# self.sp_xyz[1] = max(self.marker_sp.pose_d.position.y - self.linear_vels[1], self.position_limits[1, 0])
		# self.sp_xyz[2] = max(min(self.marker_sp.pose_d.position.z + self.linear_vels[2], self.position_limits[2, 1]), self.position_limits[2, 0])

		# # project onto workspace radius if outside workspace
		# xy_norm = np.linalg.norm(self.sp_xyz[:2])
		# if xy_norm > self.xy_max_r:
		# 	print('outside workspace!!')
		# 	# rescale onto the max radius circle
		# 	self.sp_xyz[:2] = (self.sp_xyz[:2]/xy_norm)*self.xy_max_r

		# self.marker_sp.pose_d.position = ros_numpy.msgify(Point, self.sp_xyz)

		# curr_ori = R.from_quat(ros_numpy.numpify(self.marker_sp.pose_d.orientation)) # both are in x,y,z,w order
		# r_vel = R.from_euler('x', self.angular_vels[0], degrees=False)
		# p_vel = R.from_euler('y', self.angular_vels[1], degrees=False)  
		# y_vel = R.from_euler('z', self.angular_vels[2], degrees=False)  

		# new_ori_q = (y_vel * p_vel * r_vel * curr_ori).as_quat()
		# self.marker_sp.pose_d.orientation = ros_numpy.msgify(Quaternion, new_ori_q)
		# self.marker_sp.cartesian_damping = tuple(np.concatenate((self.translation_damping, self.rotation_damping)))

		# self.marker_sp.header.stamp = rospy.Time.now()

		# self.desired_pose_viz_callback()
		# self.pose_wrench_pub.publish(self.marker_sp)
		print('not implemented')
		pass

	def start_robot(self):
		# if self.ip is None:
		# 	raise Exception('IP not provided.')
		# self.arm = XArmAPI(self.ip, is_radian=False)
		# self.arm.motion_enable(enable=False)
		# self.arm.motion_enable(enable=True)
		# if self.arm.error_code != 0:
		# 	self.arm.clean_error()
		# self.set_mode_and_state()
		print('not implemented')
		pass 

	def set_mode_and_state(self, mode=0, state=0):
		# self.arm.set_mode(mode)
		# self.arm.set_state(state=state)
		print('not implemented')
		pass 

	def clear_errors(self):
		# self.arm.clean_warn()
		# self.arm.clean_error()
		print('not implemented')
		pass 

	def has_error(self):
		print('not implemented')
		pass
		# return self.arm.has_err_warn

	def reset(self, home = False, reset_at_home=True):
		# if self.arm.has_err_warn:
		# 	self.clear_errors()
		# if home:
		# 	if reset_at_home:
		# 		self.move_to_home()
		# 	else:
		# 		self.move_to_zero()
		# 	if self.keep_gripper_closed:
		# 		self.close_gripper_fully()
		# 	else:
		# 		self.open_gripper_fully()
		print('not implemented')
		pass

	def move_to_home(self, open_gripper=False):
		# pos = self.get_position()
		# pos[0] = self.home[0]
		# pos[1] = self.home[1]
		# pos[2] = self.home[2]
		# self.set_position(pos)
		# if open_gripper and not self.keep_gripper_closed:
		# 	self.open_gripper_fully()
		print('not implemented')
		pass
	
	def set_random_pos(self):
		# self.clear_errors()
		# self.set_mode_and_state()
		# pos = self.get_position()
		
		# # Move up
		# pos[2] = self.z_limit[1]
		# self.set_position(pos)

		# # Set random pos
		# x_disp = self.low_range[0] + np.random.rand()*(self.high_range[0] - self.low_range[0])
		# y_disp = self.low_range[1] + np.random.rand()*(self.high_range[1] - self.low_range[1])
		# z_disp = self.low_range[2] + np.random.rand()*(self.high_range[2] - self.low_range[2])
		
		# pos[0] = self.home[0] + x_disp * np.random.choice([-1,1])		# Here we sample in a square ring around the home 
		# pos[1] = self.home[1] + y_disp * np.random.choice([-1,1])		# Here we sample in a square ring around the home 
		# pos[2] = self.home[2] + z_disp if not self.highest_start else self.z_limit[1] 									# For z we jsut sample from [a,b]
		# self.set_position(pos)
		# if self.keep_gripper_closed:
		# 	self.close_gripper_fully()
		# else:
		# 	self.open_gripper_fully()
		print('not implemented')
		pass 

	def move_to_zero(self):
		# pos = self.get_position()
		# pos[0] = min(max(self.x_limit[0],0), self.x_limit[1])# 0
		# pos[1] = min(max(self.y_limit[0],0), self.y_limit[1])# 0
		# pos[2] = min(max(self.z_limit[0],0), self.z_limit[1]) if not self.highest_start else self.z_limit[1] # 0
		# self.set_position(pos)

		print('not implemented')
		pass 


	def set_position(self, pos, wait=False, use_roll=False, use_pitch=False, use_yaw=False):
		# pos = self.limit_pos(pos)
		# x = (pos[0] + self.zero[0])*100
		# y = (pos[1] + self.zero[1])*100
		# z = (pos[2] + self.zero[2])*100
		# roll = pos[3] if use_roll else self.roll
		# pitch = pos[4] if use_pitch else self.pitch
		# yaw = pos[5] if use_yaw else self.yaw
		# self.arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, wait=wait)

		print('not implemented')
		pass 

	def get_position(self):
		# pos = self.arm.get_position()[1]
		# x = (pos[0]/100.0 - self.zero[0])
		# y = (pos[1]/100.0 - self.zero[1])
		# z = (pos[2]/100.0 - self.zero[2])
		# return np.array([x,y,z, pos[3], pos[4], pos[5]]).astype(np.float32)
		print('not implemented')
		pass 

	def get_gripper_position(self):
		# code, pos = self.arm.get_gripper_position()
		# if code!=0:
		# 	raise Exception('Correct gripper angle cannot be obtained.')
		# return pos
		print('not implemented')
		pass 

	def open_gripper_fully(self):
		# self.set_gripper_position(self.gripper_max_open)
		print('not implemented')
		pass 

	def close_gripper_fully(self):
		# self.set_gripper_position(self.gripper_min_open)
		print('not implemented')
		pass 

	def open_gripper(self):
		# self.set_gripper_position(self.get_gripper_position() + self.gripper_action_scale)
		print('not implemented')
		pass 

	def close_gripper(self):
		# self.set_gripper_position(self.get_gripper_position() - self.gripper_action_scale)
		print('not implemented')
		pass 

	def set_gripper_position(self, pos, wait=False):
		'''
		wait: To wait till completion of action or not
		'''
		# if pos<self.gripper_min_open:
		# 	pos = self.gripper_min_open
		# if pos>self.gripper_max_open:
		# 	pos = self.gripper_max_open
		# self.arm.set_gripper_position(pos, wait=wait, auto_enable=True)
		print('not implemented')
		pass 

	def get_servo_angle(self):
		# code, angles = self.arm.get_servo_angle()
		# if code!=0:
			# raise Exception('Correct servo angles cannot be obtained.')
		# return angles
		print('not implemented')
		pass 

	def set_servo_angle(self, angles, is_radian=None):
		'''
		angles: List of length 8
		'''
		# self.arm.set_servo_angle(angle=angles, is_radian=is_radian)
		print('not implemented')
		pass 
	
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
	# print('before spin')
	# rospy.spin()
	# print('after spin')
	# loop until ctrl-c without using rospy.spin()
	# while not rospy.is_shutdown():
		# do stuff
		# print('doing stuff')
		# time.sleep(1)
		# pass

	# arm.start_robot()
	# arm