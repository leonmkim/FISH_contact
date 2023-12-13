# FISH contact 
## Dependencies
Needs franka_ros (under my fork) branch noetic-leon-0.9.0 and contact_estimation_ros branch noetic-devel

## Testing teleop
```
pyenv activate fish_contact
source ~/FISH_franka_ros_ws/devel/setup.zsh
python3 gym-envs/gym_envs/envs/teleop/xbox_joy.py
```

## Running sim

```
source ~/FISH_franka_ros_ws/devel/setup.zsh
roslaunch fish_simulation franka_gazebo.launch

```

## Running real
