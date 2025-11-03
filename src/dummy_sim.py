#!/usr/bin/env python3
import rospy
from vader_msgs.msg import HarvestResult
import time
import random

def publisher_node():
    rospy.init_node('simulation_result_publisher')
    pub = rospy.Publisher('/harvest_status', HarvestResult, queue_size=10)
    
    # Wait for 3 seconds before publishing
    time.sleep(.1)
    
    msg = HarvestResult()
    msg.result = random.randint(0, 10)
    msg.reason = "testing" + str(msg.result)
    
    rospy.loginfo("Publishing HarvestResult message with result=%d", msg.result)
    pub.publish(msg)
    
    rospy.spin()  # Keep the node alive to allow message delivery

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
