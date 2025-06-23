#!/usr/bin/env python3
import random

PREFIX="clear; roslaunch vader_hri dualsim_noplan.launch "

def generate_poses():
    # Generate random pose
    pose = {
        "x": round(random.uniform(0.45, 0.55), 3), #depth from arm reach
        "y": round(random.uniform(0.35, 0.55), 3), #left from x-axis
        "z": round(random.uniform(0.4, 0.6), 3),
        "roll": round(random.uniform(-0.5, 0.5), 3),
        "pitch": round(random.uniform(-0.5, 0.5), 3)
    }

    arg_sim_pepper_pose = f"sim_pepper_pose:=\"{pose['x']} {pose['y']} {pose['z']} {pose['roll']} {pose['pitch']} 0.0\""

    print(PREFIX + arg_sim_pepper_pose)

if __name__ == "__main__":
    generate_poses()
