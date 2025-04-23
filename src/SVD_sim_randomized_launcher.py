#!/usr/bin/env python3
import random
import subprocess
# import os
import time

PREFIX="clear; roslaunch vader_hri vader_svd_dualsim.launch "

PEPPER_SDF_FILE = "/home/docker_ws/src/vader_sim/src/xarm_gazebo/worlds/breakable_pepper.sdf"

def generate_and_spawn():
    # Generate random pose
    pose = {
        "x": round(random.uniform(0.25, 0.45), 2),
        "y": round(random.uniform(-0.45, -0.65), 2),
        "z": round(random.uniform(0.4, 0.6), 2),
        "roll": round(random.uniform(-0.5, 0.5), 2),
        "pitch": round(random.uniform(-0.5, 0.5), 2)
        # "x": 0.25,
        # "y": -0.6,
        # "z": 0.405,
        # "roll": 0.4,
        # "pitch": 0.1
    }

    # Below pose should add 1.101 to z, 0.696 to y
    pose_gazebo = {
        "x": round(0.5-pose["x"], 3),
        "y": round(-0.51-pose["y"], 3),
        "z": round(pose["z"] + 1.025, 3),
        "roll": -pose["pitch"],
        "pitch": pose["roll"]
    }

    arg_sim_pepper_pose = f"sim_pepper_pose:=\"{pose['x']} {pose['y']} {pose['z']} {pose['roll']} {pose['pitch']} 0.0\""
    arg_gazebo_sim_pepper_pose = f"gazebo_sim_pepper_pose:=\"{pose_gazebo['x']} {pose_gazebo['y']} {pose_gazebo['z']} {pose_gazebo['roll']} {pose_gazebo['pitch']} 0.0\""

    hri_launch_command = PREFIX + arg_sim_pepper_pose + " " + arg_gazebo_sim_pepper_pose

    gazebo_model_name = "pepper1"
    gazebo_command = "rosrun gazebo_ros spawn_model -gazebo_namespace /gazebo -file "
    gazebo_command += PEPPER_SDF_FILE
    gazebo_command += f" -x {pose_gazebo['x']} -y {pose_gazebo['y']} -z {pose_gazebo['z']} -R {pose_gazebo['roll']} -P {pose_gazebo['pitch']} -sdf -model {gazebo_model_name}"

    hri_proc = subprocess.Popen([hri_launch_command], shell=True)
    gazebo_proc = subprocess.Popen([gazebo_command], shell=True)

    try:
        hri_proc.wait()
        gazebo_proc.wait()
    except KeyboardInterrupt:
        hri_proc.terminate()
        gazebo_proc.terminate()
        hri_proc.wait()
        gazebo_proc.wait()


if __name__ == "__main__":
    generate_and_spawn()