# Quaternion MPC

This branch is used for the quaternion MPC falling cat experiment.

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
git checkout falling_cat # switch to "falling_cat" branch
cd PATH_TO_YOUR_ROS_WORKSPACE
catkin build # run "catkin clean" first if your workspace has been built using another branch
echo "export PATH_TO_YOUR_ROS_WORKSPACE/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## Run the controller
First, launch the simulation:
```
roslaunch unitree_gazebo normal.launch rname:=a1_rw
```
Keep the simulation running, open another terminal, run:
```
rosrun unitree_controller unitree_servo # reset all joint angles
```
Keep `unitree_servo` running, open another terminal, run:
```
rosrun unitree_controller unitree_move_kinetic # set random initial attitude
```
Now, we are ready to run the MPC controller! In a new terminal, run:
```
roslaunch legged_ctrl ${mpc_type}_mpc.launch # mpc_type can be "quat" or "euler"
```
After running this, go back to the terminal where `unitree_move_kinetic` is running, then press `Ctrl+C`, shut it down to let the robot fall.
