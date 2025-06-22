#!/usr/bin/env python3
import random

PREFIX="clear; roslaunch vader_hri dualsim_noplan.launch "

def generate_poses():
    # Generate random pose
    pose = {
        "x": round(random.uniform(0.45, 0.55), 2), #depth from arm reach
        "y": round(random.uniform(0.35, 0.55), 2), #left from x-axis
        "z": round(random.uniform(0.4, 0.6), 2),
        "roll": round(random.uniform(-0.5, 0.5), 2),
        "pitch": round(random.uniform(-0.5, 0.5), 2)
    }

    # Below pose should add 1.101 to z, 0.696 to y
    pose_gazebo = {
        "x": round(pose["x"], 3),
        "y": round(pose["y"], 3),
        "z": round(pose["z"], 3),
        "roll": pose["roll"],
        "pitch": pose["pitch"]
    }

    arg_sim_pepper_pose = f"sim_pepper_pose:=\"{pose['x']} {pose['y']} {pose['z']} {pose['roll']} {pose['pitch']} 0.0\""
    arg_gazebo_sim_pepper_pose = f"gazebo_sim_pepper_pose:=\"{pose_gazebo['x']} {pose_gazebo['y']} {pose_gazebo['z']} {pose_gazebo['roll']} {pose_gazebo['pitch']} 0.0\""

    print(PREFIX + arg_sim_pepper_pose + " " + arg_gazebo_sim_pepper_pose)

if __name__ == "__main__":
    generate_poses()
