# dgree (디그리)

A 2-DOF articulated robot arm plotter with computer vision-based feedback control.

## Overview

dgree is a ROS2-based robot arm plotter that uses marker detection for closed-loop position control. 

## System Architecture

- **Perception Layer**: Camera driver and marker detection
- **Planning & Control**: Goal input/Path planning, Inverse Kinematics, and PID control
- **Hardware Interface**: Motor driver for actuator commands

## Packages (WIP)

- `dgree_perception` - Camera and marker detection nodes
- `dgree_control` - Planning and control nodes (IK, PID)
- `dgree_hardware` - Motor driver interface
- `dgree_launch` - Launch files and configurations
- `dgree_description` - WIP

## Usage (WIP)
