#!/usr/bin/env python3
import random

PREFIX="clear; roslaunch vader_hri vader_svd_dualsim.launch "

def generate_poses():
    # Generate random pose
    pose = {
        "x": round(random.uniform(0.25, 0.45), 2),
        "y": round(random.uniform(-0.45, -0.65), 2),
        "z": round(random.uniform(0.3, 0.5), 2),
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

    print(PREFIX + arg_sim_pepper_pose + " " + arg_gazebo_sim_pepper_pose)

if __name__ == "__main__":
    generate_poses()
