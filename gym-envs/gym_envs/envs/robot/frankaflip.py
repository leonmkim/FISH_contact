from cgitb import enable
import time
from turtle import home
import gym
from gym import spaces
from gym_envs.envs import franka_env

import numpy as np

class FrankaFlipEnv(franka_env.FrankaEnv):
	def __init__(self, height=84, width=84, step_size=10, enable_arm=True, enable_gripper=True, enable_camera=True, camera_view='side',
				 use_depth=False, dist_threshold=0.05, random_start=True, x_limit=None, y_limit=None, z_limit=None, pitch=0, roll=136, keep_gripper_closed=True, goto_zero_at_init=False):
		franka_env.FrankaEnv.__init__(
			self,
			# home_displacement=[1.7, 1.20, 1.82],
			home_displacement=[0.17, 0.120, 0.182],
			height=height,
			width=width,
			step_size=step_size,
			enable_arm=enable_arm, 
			enable_gripper=enable_gripper,
			enable_camera=enable_camera,
			camera_view=camera_view,
			use_depth=use_depth,
			keep_gripper_closed=keep_gripper_closed,
			highest_start=True,
			x_limit=x_limit,
			y_limit=y_limit,
			z_limit=z_limit,
			pitch=pitch,
			roll=roll,
			goto_zero_at_init=goto_zero_at_init
		)
		self.action_space = spaces.Box(low = np.array([-1,-1,-1, -1, -1, -1],dtype=np.float32), 
									   high = np.array([1, 1, 1, 1, 1, 1],dtype=np.float32),
									   dtype = np.float32)
		self.dist_threshold = dist_threshold
		self.random_start = random_start

		if self.random_start:
			self.random_limits = [
				[self.x_limit[0], self.x_limit[1]],
				# [1.86, self.y_limit[1]],
				# upper y_lim provided is .12, meaning this is only 1.4 cm range. This is used in the hacky looking limit_pos method below
				[0.186, self.y_limit[1]], 
				[self.z_limit[0], self.z_limit[1]]
			]
		# self.z_pos_limit = 1.1
		self.z_pos_limit = 0.11

		# Set limits for actions based on max and min in colleced data
		self.amax = np.array([0.1432122, 0.18543443, 0.23669696, 2.0728912, 2.23591, 2.1987822])
		self.amin = np.array([-0.06355667, -0.25070894, -0.16945803, -11.674248, -2.630049, -3.588492])

		self.diff = self.amax - self.amin

	def arm_refresh(self, reset=True):
		# self.arm.clear_errors()
		# self.arm.set_mode_and_state()
		if reset:
			self.arm.reset(home=True, reset_at_home=True)
		time.sleep(2)

	def reset(self):
		if not self.enable_arm:
			return np.array([0,0,0], dtype=np.float32)
		self.arm_refresh(reset=True)
		
		# for this task, this should be true
		# if self.keep_gripper_closed:
		# 	self.arm.close_gripper_fully() 
		# else:
		# 	self.arm.open_gripper_fully()

		time.sleep(0.4)		
		obs = {}
		obs['features'] = np.array(self.arm.get_position(), dtype=np.float32)
		obs['pixels'] = self.render(mode='rgb_array', width=self.width, height=self.height)
		return obs

	# get and set random pos dont actually seem to be used in reset method here. Rather just moves to home
	def get_random_pos(self, pos=None):
		x_disp = self.random_limits[0][0] + np.random.rand()*(self.random_limits[0][1] - self.random_limits[0][0])
		y_disp = self.random_limits[1][0] + np.random.rand()*(self.random_limits[1][1] - self.random_limits[1][0])
		z_disp = self.random_limits[2][0] + np.random.rand()*(self.random_limits[2][1] - self.random_limits[2][0])
		
		if pos is None:
			pos = np.zeros(3).astype(np.float32)
		pos[0] = x_disp
		pos[1] = y_disp 
		pos[2] = z_disp if not self.arm.highest_start else self.z_limit[1] 									
		return pos

	def set_random_pos(self):
		# self.arm.clear_errors()
		# self.arm.set_mode_and_state()
		pos = self.arm.get_position()
		
		# Move up
		pos[2] = self.arm.z_limit[1]
		self.arm.set_position(pos)

		# Set random pos
		pos = self.get_random_pos(pos)								

		self.arm.set_position(pos)
		# if self.arm.keep_gripper_closed:
		# 	self.arm.close_gripper_fully()
		# else:
		# 	self.arm.open_gripper_fully()

	def step(self, action):
		# if self.arm.has_error():
		# 	self.arm.clear_errors()
		# 	self.arm.set_mode_and_state()

		new_pos = self.arm.get_position()

		action = action * self.diff + self.amin
		new_pos += action

		if self.enable_arm:
			self.set_position(new_pos)
			time.sleep(0.4)

		# if self.enable_gripper:
		# 	if action[3]>0.5:
		# 		self.arm.open_gripper_fully()
		# 		time.sleep(0.2)
		# 	elif action[3]<-0.5:
		# 		self.arm.close_gripper_fully()
		# 		time.sleep(0.2)

		self.reward = 0
		
		done = False
		
		info = {}
		info['is_success'] = 1 if self.reward==1 else 0

		obs = {}
		obs['features'] = np.array(self.arm.get_position(), dtype=np.float32)
		obs['pixels'] = self.render(mode='rgb_array', width=self.width, height=self.height)

		return obs, self.reward, done, info

	def set_position(self, pos, wait=False):
		pos = self.limit_pos(pos)
		# x = (pos[0] + self.arm.zero[0])*100
		# y = (pos[1] + self.arm.zero[1])*100
		# z = (pos[2] + self.arm.zero[2])*100
		x = (pos[0] + self.arm.zero[0])
		y = (pos[1] + self.arm.zero[1])
		z = (pos[2] + self.arm.zero[2])

		self.arm.arm.set_position(x=x, y=y, z=z, roll=pos[3], pitch=pos[4], yaw=pos[5], wait=wait)

	def limit_pos(self, pos):
		# is this a hack to keep spatula near the bagel when close to the pan?
		if pos[2] <= self.z_pos_limit:
			pos[1] = max(self.random_limits[1][0], pos[1])
			pos[1] = min(self.random_limits[1][1], pos[1])
		else:
			pos[1] = max(self.y_limit[0], pos[1])
			pos[1] = min(self.y_limit[1], pos[1])
		
		pos[0] = max(self.x_limit[0], pos[0])
		pos[0] = min(self.x_limit[1], pos[0])
		
		pos[1] = max(self.y_limit[0], pos[1])
		pos[1] = min(self.y_limit[1], pos[1])

		pos[2] = max(self.z_limit[0], pos[2])
		pos[2] = min(self.z_limit[1], pos[2])

		pos[3] = min(max(110, pos[3]), 136)
		# is this part of their "guided" exploration that they just hard-coded?
		pos[4] = min(max(-2, pos[4]), 2) 
		pos[5] = min(max(-2, pos[5]), 2)

		return pos