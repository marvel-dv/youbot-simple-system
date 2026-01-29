# youbot-simple-system
Autonomous mobile manipulation system for pick-and-place tasks using a KUKA YouBot in ROS and Gazebo.

# Autonomous Pick-and-Place with a KUKA youBot

This repository contains a **ROS-based modular system for autonomous pick-and-place tasks**
using a **KUKA youBot mobile manipulator** in a simulated environment.

The project was developed as an academic proof of concept, focusing on the **integration of
perception, motion planning and control** rather than on algorithmic optimization. The system
combines visual perception based on **AprilTags**, base motion control, arm trajectory planning
using **MoveIt!**, and a supervisory node that orchestrates the complete task execution.

All experiments were conducted in **Gazebo** using a simulated YouBot model.

## Key Features

- Modular ROS architecture
- Autonomous pick-and-place execution
- Visual perception using AprilTag markers
- Omnidirectional base control with servo-based approach
- Arm motion planning with MoveIt!
- Supervisory node with task-level control and failure handling
- Fully reproducible simulation setup

## System Requirements

- Ubuntu 20.04
- ROS Noetic
- Gazebo
- MoveIt!

## Disclaimer

This project was developed and validated **exclusively in simulation** and is intended for
academic and research purposes. Additional work is required for deployment on real hardware or more reliable tasks

## Academic Context

This repository is part of an undergraduate thesis focused on mobile manipulation and system
integration using ROS.

## Installation and Setup
> **Note:** If you cloned this repository (`youbot-simple-system`), all required
> third-party packages are already included inside the `src/` directory.
> The following cloning steps are only necessary if you are setting up the
> workspace manually from scratch.
### 1. Create a workspace

mkdir -p ~/youbot_ws/src
cd ~/youbot_ws
catkin_make
source devel/setup.bash


### 2. Clone required repositories

cd ~/youbot_ws/src

- git clone https://github.com/mas-group/youbot_driver.git -b indigo-devel
- git clone https://github.com/mas-group/youbot_driver_ros_interface.git -b trajectory_controller_dirty_fix
- git clone https://github.com/mas-group/youbot_description.git -b kinetic-devel
- git clone https://github.com/mas-group/youbot_simulation.git -b noetic-devel
- git clone https://github.com/mas-group/youbot-manipulation -b kinetic
- git clone https://github.com/wnowak/brics_actuator.git -b master
- git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git



### 3. Compatibility fix (xacro)

cd ~/youbot_ws/src
grep -rl "xacro.py" . | xargs sed -i 's/xacro\.py/xacro/g'

### 4. Build the workspace
cd ~/youbot_ws
catkin_make
source devel/setup.bash

## Running the System

## General launch files

- roslaunch youbot_tools youbot_task.launch

- roslaunch youbot_tools youbot_planning.launch

- roslaunch youbot_vision youbot_perception.launch


## After all subsystems are running, start the supervisory node:
rosrun youbot_tools pick_supervisor.py

## Execute the pick-and-place task
rosservice call /execute_pick

##Useful Commands

## Base control (manual steering)
rosrun youbot_tools youbot_steering.py

## Base approach controller
Start the approach controller:

rosrun youbot_tools approach_controller.py

Call the service to approach a detected tag:

rosservice call /approach_tag "desired_distance: 0.4"

## Gripper control
Start the gripper node:

rosrun youbot_tools gripper_node.py

Available gripper commands:

- rosservice call /arm_1/gripper/open

- rosservice call /arm_1/gripper/close

- rosservice call /arm_1/gripper/grasp

- rosservice call /arm_1/gripper/fakegrasp

- rosservice call /arm_1/gripper/fakeopen

## Manipulator planning (inverse kinematics service)

rosrun youbot_tools go_to_pose_moveit_server.py

## Manipulator control GUI
rosrun youbot_tools go_to_custom_pose.py

##Perception Subsystem
Launch AprilTag detection:
roslaunch youbot_vision proto-apriltag.launch

Run the node that publishes the target pose:
rosrun youbot_vision tag_poseV2.py

## Debug and Testing Tools
- rqt_image_view
- rosrun youbot_tools detection_test.py
- rosrun youbot_tools approach_test.py
- rosrun youbot_tools planning_test.py





