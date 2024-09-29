# Quaternion MPC

This branch is used for the quaternion MPC humanoid experiment.

## Installation
- System: Ubuntu 20.04.
- Ensure that Git and ROS Noetic are properly installed.
- Necessary dependencies are installed using the bash script `install.bash` (in the `main` branch).

First, enter the following commands to install dependencies (if you are using other shells, such as zsh, please modify the commands and install.bash script accordingly). You can skip this if you have already installed them:
```
cd PATH_TO_YOUR_ROS_WORKSPACE/src
git clone https://github.com/zixinz990/quaternion-mpc.git
cd quaternion-mpc
bash ./install.bash # the installation script is in the main branch
```
Then switch to this branch and build your catkin workspace:
```
git checkout humanoid # switch to "humanoid" branch
cd PATH_TO_YOUR_ROS_WORKSPACE
catkin build # run "catkin clean" first if your workspace has been built using another branch
echo "export PATH_TO_YOUR_ROS_WORKSPACE/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Run the controller
First, launch the simulation:
```
roslaunch humanoid_gazebo normal.launch
```

Keep the simulation running, open another terminal, run:
```
rosrun humanoid_gazebo set_default_joint_pos # reset all joint angles
```

Keep `set_default_joint_pos` running, open another terminal, run:
```
rosrun humanoid_gazebo set_default_body_pose # reset body pose
```

If you see that the robot is in the right position with the right pose, press `Ctrl+C` to shut down `set_default_body_pose` node. You will see that both feet touch the ground.

In Gazebo, the simulation of foot-ground contact is not accurate, so you can observe the robot's feet slowly sliding forward. If you want to limit this, you can uncomment lines 38-90 in `humanoid_gazebo/worlds/earth.world` file. These lines create invisible blocks around each foot.

In addition, don't forget to connect your joystick to the computer, and run:
```
rosrun joy joy_node
```

Now, we are ready to run the MPC controller! In a new terminal, run:
```
roslaunch humanoid_mpc_control quat_mpc.launch
```

After running this, go back to the terminal where `set_default_joint_pos` is running, then press `Ctrl+C`, shut it down to make our MPC controller take over the control of the robot.

## TODO
Add Euler MPC into this branch.