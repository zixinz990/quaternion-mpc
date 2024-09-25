# Quaternion MPC

This repo is used for the quaternion MPC experiments.

**Robots with Attitude: Singularity-Free Quaternion-Based Model-Predictive Control for Agile Legged Robots**, Zixin Zhang, John Z. Zhang, Shuo Yang, Zachary Manchester, [pdf](https://arxiv.org/abs/2409.09940)

```
@misc{zhang2024robotsattitudesingularityfreequaternionbased,
      title={Robots with Attitude: Singularity-Free Quaternion-Based Model-Predictive Control for Agile Legged Robots}, 
      author={Zixin Zhang and John Z. Zhang and Shuo Yang and Zachary Manchester},
      year={2024},
      eprint={2409.09940},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2409.09940}, 
}
```

Branches:

- `main`: Quaternion MPC and Euler MPC for Unitree Go1 walking control. Can be deployed in Gazebo and on hardware.
- `spider_dog`: Quaternion MPC for Unitree Go1 chimney climbing control. Can be deployed in Gazebo and on hardware.
- `falling_cat`: Quaternion MPC for quadruped airborne attitude control.
- `humanoid`: Quaternion MPC and Euler MPC for MIT humanoid balancing control.

**September 25, 2024: working on fine-tuning, updating spider_dog branch, removing unnecessary code, and editing README.**

# Installation

- Run in [Ubuntu 20.04](https://releases.ubuntu.com/focal/).
- Make sure [git](https://git-scm.com/) and [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) have been installed properly.
- Necessary dependencies are installed using the bash script [install.bash](https://github.com/RoboticExplorationLab/legged_mpc_control/blob/main/install.bash).

All you need to do is enter the following commands (if you are using other shells, such as zsh, please modify the commands and [install.bash](https://github.com/RoboticExplorationLab/legged_mpc_control/blob/main/install.bash) script accordingly):
```
cd PATH_TO_YOUR_CATKIN_WORKSPACE/src
git clone https://github.com/zixinz990/quaternion-mpc.git
cd quaternion-mpc
bash ./install.bash
cd ../..
catkin build
echo "export PATH_TO_YOUR_CATKIN_WORKSPACE/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Run the Controller

If you want to run the controller in Gazebo, you need to launch the simulation first:

```
roslaunch unitree_gazebo normal.launch rname:=${robot_type}
```

If you are using a real robot, make sure it is connected to the computer and the power is on. Make sure the robot is in a safe environment. Press `L2 + B` and `L1 + L2 + START` to get the robot down to the ground.

To run the controller, enter the following command:

```
roslaunch legged_ctrl ${env_type}_${robot_type}_${controller_type}.launch
```

- `env_type` can be `gazebo` or `hardware`.
- `robot_type` can be `a1` or `go1`.
- `controller_type` can be `convex_mpc` or `quat_mpc`.

The following two commands can help you reset the robot's pose in Gazebo without restarting the simulation:

```
rosrun unitree_controller unitree_servo # reset joint angles
rosrun unitree_controller unitree_move_kinetic # reset body pose
```

# Control the Robot using a Joystick

`joy_node` will be automatically open when you launch the controller. You can do `rostopic echo /joy` to check the connection.

Default joystick settings (Xbox Controller):

| Button | Function |
| :----- | :------- |
| Left stick | Control yaw angle and body's height |
| Right stick | Control walking direction and speed|
| `A` | Switch mode between "stand" and "walk" |
| `LB` | Shut down controller |

To change this, modify the YAML files in [src/legged_ctrl/config](https://github.com/RoboticExplorationLab/legged_mpc_control/tree/main/src/legged_ctrl/config).

# Debug Instructions

1. Sometimes ROS cannot find some packages. Make sure you have sourced the `devel/setup.bash` file in `legged_mpc_control` and `unitree_ws`, and there are related commands in your `~/.bashrc` file.
2. If you have sourced `devel/setup.bash` file in `unitree_ws`, but ROS still gives error messages about Unitree packages when you build `legged_mpc_control`, try `catkin clean && catkin build && source devel/setup.bash` in `unitree_ws`.
3. [install.bash](https://github.com/RoboticExplorationLab/legged_mpc_control/blob/main/install.bash) automatically installs CMake 3.24.3, because the program requires CMake 3.23+. If you think this will mess up your computer, feel free to delete the related commands in [install.bash](https://github.com/RoboticExplorationLab/legged_mpc_control/blob/main/install.bash). But to use the controller, you need to make sure you have CMake 3.23+ installed.

# Development Guide

Any contribution is welcome. Please ensure that the following naming rules are followed:

## Naming rules in `legged_ctrl` package
1. The name of **folders** should be **lowercase**. Use underscores to connect words, eg., `my_folder`, etc.
2. The name of **C++ header and source files** should follow the **PascalCase** rule (also known as **UpperCamelCase**), eg., `MyMpc.h`, etc.
3. The name of **XML-based files** and **YAML files** should be **lowercase**. Use underscores to connect words, eg., `my_launch_file.launch`.
4. The name of **classes, structures and enumerated types** should follow the **PascalCase** rule, eg., `MyMpc`, etc.
5. The name of **namespace, variables, parameters, enumerators and functions** should be **lowercase**. Use underscores to connect words, eg., `my_namespace`.
6. The name of macros or constants appear in the **preprocessor** should be **uppercase**. Use underscores to connect words, eg., `#define MY_CONST 1`.

# Acknowledgement

[Dr. Shuo Yang](https://github.com/ShuoYangRobotics) made major contributions to the development of the first version of this codebase (September 2022), particularly in the development of the state estimators and the code structure. [John Ziyang Zhang](https://github.com/johnzhang3) made significant contributions to the chimney climbing experiment. The solver was developed based on [Dr. Brian E. Jackson](https://github.com/bjack205)'s C++ implementation of [ALTRO](https://github.com/bjack205/altro.git). Thank you all for your support. Finally, special thanks to [Dr. Zachary Manchester](https://github.com/zacmanchester) for all the insightful guidance along the way.
