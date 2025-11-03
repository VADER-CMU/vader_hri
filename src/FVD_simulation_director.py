#!/usr/bin/env python3
import os
import rospy
from vader_msgs.msg import HarvestResult
import subprocess
import threading
import signal
import matplotlib.pyplot as plt
from collections import Counter

class SimulationDirector:
    def __init__(self):
        rospy.init_node('harvest_status_listener', anonymous=True)
        self.sub = rospy.Subscriber('/harvest_status', HarvestResult, self.callback)
        self.message_received = threading.Event()
        self.process = None
        self.received_msg = None
        self.run_count = 0
        self.max_runs = 2

        self.results = []  # Will hold result codes (e.g., 0 for failure, 1 for success)
        self.reasons = []  # Will hold reasons for failure, etc.
    def callback(self, msg):
        self.received_msg = msg
        self.results.append(self.received_msg.result)
        self.reasons.append(self.received_msg.reason)
        rospy.loginfo("Message received on /harvest_status: %i, failure reason (if any): %s", self.received_msg.result, self.received_msg.reason)
        self.message_received.set()

    def run(self):
        pepper_x = 1.0 #m
        pepper_y = 0.15 #m 
        pepper_z = 0.4 #m

        env = os.environ.copy()
        env['DISABLE_ROS1_EOL_WARNINGS'] = '1'
        while not rospy.is_shutdown() and self.run_count < self.max_runs:
            self.message_received.clear()
            rospy.loginfo("Starting run %i", self.run_count)

            #Test the tests
            # self.process = subprocess.Popen(['rosrun', 'vader_hri', 'dummy_sim.py'])


            self.process = subprocess.Popen(
                ['rosrun', 'vader_hri', 'SVD_sim_randomized_launcher.py', '--x', str(pepper_x), '--y', str(pepper_y), '--z', str(pepper_z)],
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
        self.plot_results(pepper_x, pepper_y, pepper_z)

    def plot_results(self, x, y, z):
        # Bin according to success (100) / failure (other values)
        success_count = self.results.count(100)

        print("%i out of %i runs succeeded" % (success_count, self.run_count))

        # Plot reasons distribution if needed
        title = 'Failure Reason Distribution for pepper (%.2f, %.2f, %.2f), Success Rate: %.2f%%' % (x, y, z, success_count / self.run_count * 100 if self.run_count > 0 else 0)
        reason_counter = Counter(self.reasons)
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
