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
