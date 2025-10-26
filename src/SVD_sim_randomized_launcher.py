#!/usr/bin/env python3
import random
import subprocess
# import os
import time
from vader_msgs.msg import SimulationPepperSequence, SimulationPopPepper
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import rospy

PREFIX="clear; roslaunch vader_hri vader_fvd_dualsim.launch "

PEPPER_SDF_FILE = "/home/docker_ws/src/vader_sim/src/xarm_gazebo/worlds/breakable_pepper.sdf"

pepper_base_poses = [
    # [0.2, -0.5, 0.5],
    # [0.5, -0.6, 0.4],
    # [0.35, -0.55, 0.65],
    [0.9, 0.25, 0.4]
]

def randomize_pepper(pepper_base_pose):
    # Randomize the pose of the pepper robot
    x = round(random.uniform(pepper_base_pose[0] - 0.05, pepper_base_pose[0] + 0.05), 2)
    y = round(random.uniform(pepper_base_pose[1] - 0.05, pepper_base_pose[1] + 0.05), 2)
    z = round(random.uniform(pepper_base_pose[2] - 0.05, pepper_base_pose[2] + 0.05), 2)
    roll = round(random.uniform(-0.5, 0.5), 2)
    pitch = round(random.uniform(-0.5, 0.5), 2)

    pepper_pose_dict = {
        "x": x,
        "y": y,
        "z": z,
        "roll": roll,
        "pitch": pitch
    }

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quaternion = quaternion_from_euler(roll, pitch, 0)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]


    return pepper_pose_dict, pose

def get_gazebo_pose(pepper_pose):
    # Convert the pepper pose to gazebo pose
    gazebo_pose = {
        "x": round(pepper_pose["x"], 3),
        "y": round(pepper_pose["y"], 3),
        "z": round(pepper_pose["z"], 3),
        "roll": pepper_pose["roll"],
        "pitch": pepper_pose["pitch"]
    }
    return gazebo_pose

def generate_and_spawn():
    randomized_peppers = []
    pepper_poses = []
    for pepper in pepper_base_poses:
        pose_dict, pose_msg = randomize_pepper(pepper)
        randomized_peppers.append(pose_dict)
        pepper_poses.append(pose_msg)

    # pose = randomize_pepper(pepper_base_poses[0])
    pose_gazebo = get_gazebo_pose(randomized_peppers[0])

    arg_sim_pepper_pose = f"sim_pepper_pose:=\"{randomized_peppers[0]['x']} {randomized_peppers[0]['y']} {randomized_peppers[0]['z']} {randomized_peppers[0]['roll']} {randomized_peppers[0]['pitch']} 0.0\""
    arg_gazebo_sim_pepper_pose = f"gazebo_sim_pepper_pose:=\"{pose_gazebo['x']} {pose_gazebo['y']} {pose_gazebo['z']} {pose_gazebo['roll']} {pose_gazebo['pitch']} 0.0\""

    hri_launch_command = PREFIX + arg_sim_pepper_pose + " " + arg_gazebo_sim_pepper_pose

    
    gazebo_spawn_procs = []
    for i in range(len(pepper_base_poses)):
        pose_gazebo = get_gazebo_pose(randomized_peppers[i])
        gazebo_model_name = "pepper" + str(i)
        gazebo_command = "rosrun gazebo_ros spawn_model -gazebo_namespace /gazebo -file "
        gazebo_command += PEPPER_SDF_FILE
        gazebo_command += f" -x {pose_gazebo['x']} -y {pose_gazebo['y']} -z {pose_gazebo['z']} -R {pose_gazebo['roll']} -P {pose_gazebo['pitch']} -sdf -model {gazebo_model_name}"
        gazebo_spawn_procs.append(subprocess.Popen([gazebo_command], shell=True))
    # gazebo_model_name = "pepper1"
    # gazebo_command = "rosrun gazebo_ros spawn_model -gazebo_namespace /gazebo -file "
    # gazebo_command += PEPPER_SDF_FILE
    # gazebo_command += f" -x {pose_gazebo['x']} -y {pose_gazebo['y']} -z {pose_gazebo['z']} -R {pose_gazebo['roll']} -P {pose_gazebo['pitch']} -sdf -model {gazebo_model_name}"

    hri_proc = subprocess.Popen([hri_launch_command], shell=True)

    rospy.init_node("SVD_sim_randomized_launcher", anonymous=True)
    time.sleep(1)
    pepper_seq_pub = rospy.Publisher("/pepper_sequence", SimulationPepperSequence, queue_size=10)
    for i in range(3):
        time.sleep(.5)
        pepper_seq_msg = SimulationPepperSequence()
        pepper_seq_msg.pepper_poses = pepper_poses
        pepper_seq_pub.publish(pepper_seq_msg)

    # gazebo_proc = subprocess.Popen([gazebo_command], shell=True)

    try:
        hri_proc.wait()
        for proc in gazebo_spawn_procs:
            proc.wait()
        # gazebo_proc.wait()
    except KeyboardInterrupt:
        hri_proc.terminate()
        for proc in gazebo_spawn_procs:
            proc.terminate()
        # gazebo_proc.terminate()
        hri_proc.wait()
        for proc in gazebo_spawn_procs:
            proc.wait()
        # gazebo_proc.wait()


if __name__ == "__main__":
    generate_and_spawn()

# #!/usr/bin/env python3
# import random

# PREFIX="clear; roslaunch vader_hri vader_fvd_dualsim.launch "

# def generate_poses():
#     # Generate random pose
#     pose = {
#         "x": round(random.uniform(0.45, 0.55), 3), #depth from arm reach
#         "y": round(random.uniform(0.35, 0.55), 3), #left from x-axis
#         "z": round(random.uniform(0.4, 0.6), 3),
#         "roll": round(random.uniform(-0.5, 0.5), 3),
#         "pitch": round(random.uniform(-0.5, 0.5), 3)
#     }

#     arg_sim_pepper_pose = f"sim_pepper_pose:=\"{pose['x']} {pose['y']} {pose['z']} {pose['roll']} {pose['pitch']} 0.0\""

#     print(PREFIX + arg_sim_pepper_pose)

# if __name__ == "__main__":
#     generate_poses()
