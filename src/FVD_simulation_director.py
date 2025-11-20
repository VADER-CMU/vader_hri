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
import random
import argparse
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
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
        self.max_runs = 100

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

            pepper_x = round(random.uniform(0.82, 1.05), 2) #m
            pepper_y = round(random.uniform(0, 0.5), 2) #m 
            pepper_z = round(random.uniform(0.3, 0.6), 2) #m
            pepper_r = round(random.uniform(-0.2, 0.2), 2) #0.1 #rad
            pepper_p = round(random.uniform(-0.2, 0.2), 2) #0.0 #rad
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
        self.export_csv()
        self.plot_results()

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

def load_runs_from_csv(path):
    runs = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row.get('x', 0) or 0)
                y = float(row.get('y', 0) or 0)
                z = float(row.get('z', 0) or 0)
                roll = float(row.get('roll', 0) or 0)
                pitch = float(row.get('pitch', 0) or 0)
                result = int(row.get('result', -1) or -1)
                reason = row.get('reason', '') or ''
            except Exception:
                continue
            runs.append(SimulationRun(x=x, y=y, z=z, roll=roll, pitch=pitch, result=result, reason=reason))
    return runs

def bin_and_count(runs, value_getter, bin_size=0.05):
    # bins: bin_value -> {'success': count, 'failures': {reason: count}}
    binned_data = defaultdict(lambda: {'success': 0, 'failures': defaultdict(int)})

    for run in runs:
        val = value_getter(run)
        bin_val = round(val / bin_size) * bin_size
        if run.result == 100:
            binned_data[bin_val]['success'] += 1
        else:
            binned_data[bin_val]['failures'][run.reason] += 1
    return binned_data

def plot_binned_data(binned_data, xlabel, title):
    bins = sorted(binned_data.keys())
    all_reasons = set()
    for data in binned_data.values():
        all_reasons.update(data['failures'].keys())
    all_reasons = sorted(all_reasons)

    fail_counts_per_reason = [
        [binned_data[bin].get('failures', {}).get(reason, 0) for bin in bins] for reason in all_reasons
    ]
    success_counts = [binned_data[bin]['success'] for bin in bins]

    x = np.arange(len(bins))
    bottom = np.zeros(len(bins))

    plt.figure(figsize=(14, 7))

    for counts, reason in zip(fail_counts_per_reason, all_reasons):
        plt.bar(x, counts, bottom=bottom, label=f'Fail: {reason}', width=0.8)
        bottom += counts

    plt.bar(x, success_counts, bottom=bottom, color='green', label='Success', width=0.8)

    plt.xlabel(xlabel)
    plt.ylabel('Count of runs')
    plt.title(title)
    plt.xticks(x, [f'{bin:.2f}' for bin in bins], rotation=90)
    plt.legend()
    plt.tight_layout()
    plt.show()

def analyze_and_plot_all(runs, bin_size=0.05):
    # Define functions to extract the values for each case
    extractors = {
        'X': lambda r: r.x,
        'Y': lambda r: r.y,
        'Z': lambda r: r.z,
        'Angle Magnitude (roll+pitch)': lambda r: np.sqrt(r.roll**2 + r.pitch**2)
    }

    for label, extractor in extractors.items():
        binned_data = bin_and_count(runs, extractor, bin_size=bin_size)
        plot_binned_data(
            binned_data,
            xlabel=f'{label} binned (step={bin_size})',
            title=f'Simulation Run Outcomes by Binned {label}'
        )

# Example usage after loading runs:
# analyze_and_plot_all(runs)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simulation director runner / plotter')
    parser.add_argument('-i', '--input-csv', help='Optional CSV file to load simulation results and plot', default=None)
    args = parser.parse_args()

    if args.input_csv:
        runs = load_runs_from_csv(args.input_csv)
        analyze_and_plot_all(runs)

    else:
        listener = SimulationDirector()
        listener.run()
