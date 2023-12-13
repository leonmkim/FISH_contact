# FISH contact 
## Dependencies
Needs franka_ros (under my fork) branch noetic-leon-0.9.0 and contact_estimation_ros branch noetic-devel

## Running hardware
On RT-kernel machine: 
```
roslaunch fish_franka franka_hw.launch
```
On this machine:
```
pyenv activate fish_contact
source ~/FISH_contact/scripts/ros_dair_machine.zshrc
rosrun rviz rviz -d ~/FISH_franka_ros_ws/src/contact_estimation_ros/fish_franka/config/franka_description_with_marker.rviz
```
## Testing teleop
```
source ~/FISH_franka_ros_ws/devel/setup.zsh
pyenv activate fish_contact
python3 gym-envs/gym_envs/envs/teleop/xbox_joy.py
```

## Running sim

```
source ~/FISH_franka_ros_ws/devel/setup.zsh
roslaunch fish_simulation franka_gazebo.launch

```
