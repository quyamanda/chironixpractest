#!/usr/bin/env python  
import sys
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import math
import time
import tf
import csv
# import rosbag
global csvfile

def callback_pose(msg):
	curr_x = msg.pose.pose.position.x
	curr_y = msg.pose.pose.position.y
	#formulate a quaternion as a list
	quaternion = (
	msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y,
	msg.pose.pose.orientation.z,
	msg.pose.pose.orientation.w)
	#convert the quaternion to roll-pitch-yaw
	rpy = tf.transformations.euler_from_quaternion(quaternion)
	curr_yaw = math.degrees(rpy[2])
	global csvfile
	filewriter = csv.writer(csvfile, delimiter = ',')	
	filewriter.writerow([time.localtime(),curr_x, curr_y, curr_yaw])

def recordpose(filepath):
	global csvfile
	csvfile = open(filepath, 'a+')
	
	# bag = rosbag.Bag('mv2goal-amcl_pose.bag') #start recording /amcl_pose
	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback_pose)

if __name__ == '__main__':
	rospy.init_node('map_navigation', anonymous=False)
	filepath = sys.argv[1]
	recordpose(filepath)
	rospy.spin()
	
