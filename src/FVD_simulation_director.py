#!/usr/bin/env python3
import os
import rospy
from vader_msgs.msg import HarvestResult
import subprocess
import threading
import signal
import matplotlib.pyplot as plt
from collections import Counter
from dataclasses import dataclass
import csv

@dataclass
class SimulationRun:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    result: int
    reason: str


class SimulationDirector:
    def __init__(self):
        rospy.init_node('harvest_status_listener', anonymous=True)
        self.sub = rospy.Subscriber('/harvest_status', HarvestResult, self.callback)
        self.message_received = threading.Event()
        self.process = None
        self.received_msg = None
        self.run_count = 0
        self.max_runs = 2

        self.runs = []  # Will hold SimulationRun instances
    def callback(self, msg):
        self.received_msg = msg
        self.runs[-1].result = self.received_msg.result
        self.runs[-1].reason = self.received_msg.reason
        rospy.loginfo("Message received on /harvest_status: %i, failure reason (if any): %s", self.received_msg.result, self.received_msg.reason)
        self.message_received.set()

    def run(self):
        env = os.environ.copy()
        env['DISABLE_ROS1_EOL_WARNINGS'] = '1'
        while not rospy.is_shutdown() and self.run_count < self.max_runs:
            self.message_received.clear()
            rospy.loginfo("Starting run %i", self.run_count)

            #Test the tests
            # self.process = subprocess.Popen(['rosrun', 'vader_hri', 'dummy_sim.py'])

            pepper_x = 1.0 #m
            pepper_y = 0.15 #m 
            pepper_z = 0.4 #m
            pepper_r = 0.1 #rad
            pepper_p = 0.0 #rad
            run = SimulationRun(
                x=pepper_x,
                y=pepper_y,
                z=pepper_z,
                roll=pepper_r,
                pitch=pepper_p,
                result=-1,
                reason=""
            )
            self.runs.append(run)
            

            self.process = subprocess.Popen(
                ['rosrun', 'vader_hri', 'SVD_sim_randomized_launcher.py', '--x', str(pepper_x), '--y', str(pepper_y), '--z', str(pepper_z), "--roll", str(pepper_r), "--pitch", str(pepper_p)],
                preexec_fn=os.setsid,
                env=env,
            )

            # Wait until a message is received
            self.message_received.wait()
            
            # Print the message
            # rospy.loginfo("Received message: %i", self.received_msg)
            
            # Send Ctrl+C (SIGINT) to subprocess
            rospy.loginfo("Terminating subprocess...")


            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait()  # Wait for subprocess to exit
            
            self.run_count += 1
        self.plot_results()
        self.export_csv()

    def export_csv(self):
        with open('simulation_results.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'z', 'roll', 'pitch', 'result', 'reason'])
            for run in self.runs:
                writer.writerow([run.x, run.y, run.z, run.roll, run.pitch, run.result, run.reason])
        rospy.loginfo("Simulation results exported to simulation_results.csv")

    def plot_results(self):
        # Bin according to success (100) / failure (other values)
        success_count = sum(1 for run in self.runs if run.result == 100)

        print("%i out of %i runs succeeded" % (success_count, self.run_count))

        # Plot reasons distribution if needed
        title = 'Failure Reason Distribution, Success Rate: %.2f%%' % (success_count / self.run_count * 100 if self.run_count > 0 else 0)
        reason_counter = Counter(run.reason for run in self.runs)
        plt.figure()
        plt.bar(reason_counter.keys(), reason_counter.values())
        plt.title(title)
        plt.ylabel('Count')
        plt.xlabel('Reasons')
        plt.xticks(rotation=45)
        plt.show()
if __name__ == '__main__':
    listener = SimulationDirector()
    listener.run()
