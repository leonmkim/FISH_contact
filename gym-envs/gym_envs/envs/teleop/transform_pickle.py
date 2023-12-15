#%%
import pickle
import numpy as np
import cv2
import os
from pathlib import Path
#%%
# user_home_path = Path(os.environ['HOME'])
# path_to_FISH_root = user_home_path / 'FISH'
# root_dir = path_to_FISH_root / 'teleop/data/Reach/'
root_dir = Path('/home/leonmkim/FISH_contact/data/Reach/2023_12_13_23_6_test')

# #%%
# file_name = f"{root_dir}/{0}/traj.pickle"

# with open(file_name, 'rb') as f:
# 	traj = pickle.load(f)

# #%%
# actions = np.array(traj['action'])
# #%%
# config_file_name = f"{root_dir}/config.pickle"

# with open(config_file_name, 'rb') as f:
# 	config = pickle.load(f)

# #%%
max_action = 0.025 # TODO: replace this by getting it from experiment config!!
num_traj = 1
use_depth = False
image_height = 84
image_width = 84

images_list = []
states_list = []
actions_list = []
rewards_list = []
for index in range(0,num_traj):
	file_name = f"{root_dir}/{index}/traj.pickle"

	with open(file_name, 'rb') as f:
		traj = pickle.load(f)

	images = np.array(traj['image_observation'], dtype=np.uint8)
	resized_images = []
	for img in images:
		resized_images.append(cv2.resize(img, (image_width, image_height), interpolation=cv2.INTER_AREA))
	images = np.array(resized_images, dtype=np.uint8)
	images = np.transpose(images, (0,3,1,2))
	if not use_depth:
		images = images[:, :3]
	images_list.append(images)
	states_list.append(np.array(traj['state_observation'], dtype=np.float32))
	action = np.array(traj['action'], dtype=np.float32)

	action[:,:3] /= max_action # just for normalizing the actions in position (max action is hardcoded here as 0.25 but it isn't necessarily )
	actions_list.append(action)
	rewards_list.append(np.array(traj['reward'], dtype=np.float32))

images_list = np.array(images_list)
states_list = np.array(states_list)
actions_list = np.array(actions_list)
rewards_list = np.array(rewards_list)

SAVE_PATH = root_dir / f'expert_demos_{image_height}.pkl'
with open(SAVE_PATH, 'wb') as outfile:
	pickle.dump([images_list, states_list, actions_list, rewards_list], outfile)
print("Saved.")