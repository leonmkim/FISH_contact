#%%
import time
import pygame

pygame.init()
joy = pygame.joystick.Joystick(0)
joy.init()
#%%

# get event for 5 seconds
start_time = time.time()
duration = 20
while time.time() - start_time < duration:
	events = pygame.event.get()
	for event in events:
		# print(event)
		if event.type == pygame.JOYAXISMOTION: #thumbsticks, 0-1 is the left (horiz, vert) and 3-4 is the right (horiz, vert)
			print(event.axis, event.value)
		elif event.type == pygame.JOYBUTTONDOWN: #means pressing button down 0-3 (A, B, X, Y), 4-5 (LB, RB), 6-7 (back, start), 8 (xbox button)
			print(event.button)
		elif event.type == pygame.JOYBUTTONUP: #means releasing button
			print(event.button)
		elif event.type == pygame.JOYHATMOTION: #dpad, pair of -1,0,1 where (right, up) is (1,1) 
			print(event.hat, event.value)
		# else:
			# print(event)
# %%
