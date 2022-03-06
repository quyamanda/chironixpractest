#!/usr/bin/env python
#open-loop control
import sys
import time
import rospy
from geometry_msgs.msg import Twist

def clockwise():
    #create new publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #initialize node
    rospy.init_node('clockwise', anonymous=True)
    #set loop rate
    rate = rospy.Rate(50) # Hz
    
    # set values for cmd_vel message
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(sys.argv[1])
    
    #keep publishing for 10s or Ctrl-C is pressed
    t0 = time.time()
    while (time.time() - t0 < 10):
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        clockwise()
    except rospy.ROSInterruptException:
        pass
