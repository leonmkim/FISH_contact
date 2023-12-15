#%%
from pathlib import Path
import os 
import pickle
import numpy as np
import moviepy.editor as mpy
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#%%
path_to_expert_demo = '/home/leonmkim/FISH_contact/data/Reach/2023_12_13_23_6_test/expert_demos_84.pkl'
with open(path_to_expert_demo, 'rb') as f:
    expert_demos = pickle.load(f)
#%%
images = expert_demos[0][0] # T x 3 x 84 x 84
states = expert_demos[1][0]
actions = expert_demos[2][0]
rewards = expert_demos[3][0]
#%%
# generate a video from the images
images = np.transpose(images, (0,2,3,1))
# convert bgr to rgb
images = images[:,:,:,::-1]
clip = mpy.ImageSequenceClip(list(images), fps=2)
clip.write_videofile("movie.mp4")

#%%
# plot the states in 3D
fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
# set a color to indicate the time step
for i in range(len(states)):
    ax.scatter(states[i,0], states[i,1], states[i,2], c=[i/len(states), 0, 0])
# ax.scatter(states[:,0], states[:,1], states[:,2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# set xlim, ylim, zlim
ax.set_xlim3d(0, 0.5)
ax.set_ylim3d(-0.5, 0.5)
ax.set_zlim3d(0, 0.3)
plt.show()
#%%
# plot the actions in 3D
fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.scatter(actions[:,0], actions[:,1], actions[:,2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# set xlim, ylim, zlim
ax.set_xlim3d(-1, 1)
ax.set_ylim3d(-1, 1)
ax.set_zlim3d(-1, 1)
plt.show()
#%%