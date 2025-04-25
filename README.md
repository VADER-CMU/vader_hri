# VADER HRI
## Attribution

This repository belongs to Carnegie Mellon University, Masters of Science - Robotic Systems Development (MRSD) Team E - VADER

Team Members: Tom Gao, Abhishek Mathur, Rohit Satishkumar, Kshitij Bhat, Keerthi GV 

First Revision: March 2025

## Introduction and Overview

HRI stands for Human-Robot Interface, and in the context of VADER, the HRI comprises of the State Machine (which controls the flow of perception - planning - extraction - storage loop) and the User Interface (which would receive user commands, switch between autonomy and teleoperation, and display critical data).

The SVD_SingleReal_HRI and SVD_DualSim_HRI programs control the harvesting sequence of the single arm setup (with gripper) with the real hardware as well as the dual arm setup (with gripper/cutter) in simulation.

Additionally, the SVD_sim_randomized_launcher.py script randomizes the pose of the pepper within the reachable simulator workspace, and creates a command that launches the Dual arm HRI and related programs.

## Usage

This package should be downloaded and built along with all other prerequisite packages. Refer to the documentation page on the project website for more information.

To run the single arm program, do:

```bash
roslaunch vader_hri vader_svd_singleReal.launch
```

To randomize and run the dual arm program, do:

```bash
rosrun vader_hri SVD_sim_randomized_launcher.py | bash
```
