import time
import pygame

# import from ../hardware/franka.py Franka
from gym_envs.envs.hardware.franka import Franka


class XboxJoy:

	def __init__(self,
				 arm,
				 scale_factor_pos=0.03, 
				 scale_factor_gripper=300, 
				 scale_factor_rotation=120, #0.1,
				 motion_scale_change=0.01,
				 random_start=False,
				 sleep=0.6, # arbitrary amount of time to sleep after each action to ensure desired goal is reached
				 is_radian=False):

		pygame.init()
		self.joy = pygame.joystick.Joystick(0)
		self.joy.init()
		# self.goal_pos = [1.73,0.09,0.2] # not used
		self.global_mov = [-1,5,0] # just used to display position in the terminal
		self.random_start = random_start
		
		self.arm = arm
		self.scale_factor_pos = scale_factor_pos
		self.scale_factor_gripper = scale_factor_gripper
		self.scale_factor_rotation = scale_factor_rotation
		self.motion_scale_change = motion_scale_change
		self.sleep = sleep
		self.is_radian = is_radian

		self.forward = False
		self.backward = False
		self.left = False
		self.right = False
		self.up = False
		self.down = False
		self.close_gripper = False
		self.open_gripper = False
		self.start = False
		self.stop = False
		self.cancel = False
		self.go_home = False
		self.bring_arm_up = False
		self.bring_arm_down = False
		self.rotate_arm_cw = False
		self.rotate_arm_ccw = False
		self.motion_scale = 0

	def init_arm(self, reset=True):
		self.arm = Franka()
		self.arm.start_robot()

		if reset:
			self.arm.reset(home=True)
			time.sleep(2)

	def detect_event(self):
		events = pygame.event.get()
		for event in events:
			if event.type == pygame.JOYAXISMOTION:
				if event.axis==1 and event.value>0.5:
					self.forward = True
				elif event.axis==1 and event.value<-0.5:
					self.backward = True
				elif event.axis==1:
					self.forward = False
					self.backward = False
				elif event.axis==0 and event.value>0.5: # this is actually the right direction on the left thumbstick
					self.left = True
				elif event.axis==0 and event.value<-0.5: # this is actually the left direction on the left thumbstick
					self.right = True
				elif event.axis==0:
					self.left = False
					self.right = False
				elif event.axis==4 and event.value<-0.5: # this is up direction on the right thumbstick
					self.up = True
				elif event.axis==4 and event.value>0.5: # this is down direction on the right thumbstick
					self.down = True
				elif event.axis==4:
					self.up = False
					self.down = False
				# elif event.axis==3 and event.value<-0.5: # this is left direction on the right thumbstick
				# 	self.rotate_arm_cw = True
				# elif event.axis==3 and event.value>0.5: # this is right direction on the right thumbstick
				# 	self.rotate_arm_ccw = True
				# elif event.axis==3:
				# 	self.rotate_arm_cw = False
				# 	self.rotate_arm_ccw = False

			if event.type == pygame.JOYBUTTONDOWN:
				if event.button == 0:			# A
					print("Close gripper")
					self.close_gripper = True
				elif event.button == 1:			# B
					print("Open gripper")
					self.open_gripper = True
				elif event.button == 7:			# start
					print("Start")
					self.start = True
				elif event.button == 6:			# back
					print("Stop")
					self.stop = True
				elif event.button == 2:			# X
					print("Go Home")
					self.go_home = True
				elif event.button == 3:			# Y
					print("Cancel demo")
					self.cancel = True
				elif event.button == 4:			# LB
					print("Decrease motion scale")
					self.scale_factor_pos = max(0.01, self.scale_factor_pos-self.motion_scale_change)
				elif event.button == 5:			# RB
					print("Increase motion scale")
					self.scale_factor_pos += self.motion_scale_change
			
			elif event.type == pygame.JOYHATMOTION:
				if event.hat == 0 and event.value == (0,1):
					print("Reset arm")
					self.arm.reset(home=True)

	def move(self):
		# if self.arm.has_error():
		# 	self.arm.clear_errors()
		# 	self.arm.set_mode_and_state()
		# 	self._move_up()
		pos, action = None, None
		if self.forward:
			pos = self._move_forward()
			action = 'move_forward'
			self.global_mov[0] += 1
			print(f"global_mov:{self.global_mov}")
		elif self.backward:
			pos = self._move_backward()
			action = 'move_backward'
			self.global_mov[0] -= 1
			print(f"global_mov:{self.global_mov}")
		elif self.left:
			pos = self._move_left()
			action = 'move_left'
			self.global_mov[1] += 1
			print(f"global_mov:{self.global_mov}")
		elif self.right:
			pos = self._move_right()
			action = 'move_right'
			self.global_mov[1] -= 1
			print(f"global_mov:{self.global_mov}")
		elif self.up:
			pos = self._move_up()
			action = 'move_up'
			self.global_mov[2] += 1
			print(f"global_mov:{self.global_mov}")
		elif self.down:
			pos = self._move_down()
			action = 'move_down'
			self.global_mov[2] -= 1
			print(f"global_mov:{self.global_mov}")
		elif self.close_gripper:
			pos = self._close_gripper()
			self.close_gripper = False
			action = 'close_gripper'
			print(f"global_mov:{self.global_mov}")
		elif self.open_gripper:
			pos = self._open_gripper()
			self.open_gripper = False
			action = 'open_gripper'
			print(f"global_mov:{self.global_mov}")
		elif self.go_home:
			pos = self.arm.move_to_home(open_gripper=False)
			self.go_home = False
			action = 'go_home'
			self.global_mov = [-1,5,0]
			print(f"global_mov:{self.global_mov}")
		elif self.start:
			pos = None
			action = 'start'
			self.start = False
		elif self.stop:
			pos = None
			action = 'stop'
			self.stop = False
		elif self.cancel:
			pos = None
			action = 'cancel'
			self.cancel = False
		elif self.rotate_arm_cw:
			pos = self._rotate_arm_cw()
			action = 'rotate_arm_cw'
		elif self.rotate_arm_ccw:
			pos = self._rotate_arm_ccw()
			action = 'rotate_arm_ccw'

		return pos, action		
	
	def _move_forward(self):
		pos = self.arm.get_position()
		pos[0] += self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos
		
	def _move_backward(self):
		pos = self.arm.get_position()
		pos[0] -= self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos

	def _move_left(self):
		pos = self.arm.get_position()
		pos[1] += self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos	

	def _move_right(self):
		pos = self.arm.get_position()
		pos[1] -= self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)	
		return pos	

	def _move_up(self):
		pos = self.arm.get_position()
		pos[2] += self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos

	def _move_down(self):
		pos = self.arm.get_position()
		pos[2] -= self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos
	
	def _roll_arm_cw(self):
		pos = self.arm.get_position()
		pos[3] += self.scale_factor_rotation
		self.arm.set_position(pos, use_roll=True)
		time.sleep(self.sleep)
		return pos
	
	def _roll_arm_ccw(self):
		pos = self.arm.get_position()
		pos[3] -= self.scale_factor_rotation
		self.arm.set_position(pos, use_roll=True)
		time.sleep(self.sleep)
		return pos

	def _open_gripper(self):
		self.arm.open_gripper_fully()
		pos = 1
		time.sleep(self.sleep/2)
		return pos	

	def _close_gripper(self):
		self.arm.close_gripper_fully()
		pos = -1
		time.sleep(self.sleep/2)
		return pos
	

	def _rotate_arm_cw(self): # Leon: actuates the 7th joint which rotates the end-effector clockwise
		angles = self.arm.get_servo_angle()
		angles[6] -= self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[6]

	def _rotate_arm_ccw(self): # Leon: actuates the 7th joint which rotates the end-effector counter-clockwise
		angles = self.arm.get_servo_angle()
		angles[6] += self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[6]

	# Leon: these aren't even used in the original code
	def _bring_arm_up(self): # Leon: actuates the 6th joint which pitches the end-effector up
		angles = self.arm.get_servo_angle()
		angles[5] -= self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[5]

	def _bring_arm_down(self): # Leon: actuates the 6th joint which pitches the end-effector down
		angles = self.arm.get_servo_angle()
		angles[5] += self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[5]

if __name__ == "__main__":
	import rospy
	home_displacement = [0.2,0,0.2]
	x_limit = [0.075, 0.35] 
	y_limit = [-0.17, 0.13]
	z_limit = [0.14, 0.34] 
	arm = Franka(
		home_displacement=home_displacement,
		x_limit=x_limit,
		y_limit=y_limit,
		z_limit=z_limit,
		)
	arm.start_robot()
	joy = XboxJoy(arm, scale_factor_pos=0.015,
			  scale_factor_gripper=50, 
			  scale_factor_rotation=10,
			  motion_scale_change=0.003,
			  sleep=0.4)
	# loop but catch ctrl+c
	while not rospy.is_shutdown():
		joy.detect_event()
		pos, action = joy.move()
		# time.sleep(0.5)