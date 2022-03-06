#!/usr/bin/env python  
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import tf
import csv
import math

#this method will make the robot move to the goal location
def move_to_goal(goalx,goaly,goalyaw):

	#define a client for to send goal requests to the move_base server through a SimpleActionClient
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	#wait for the action server to come up
	while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		rospy.loginfo("Waiting for the move_base action server to come up")

	goal = MoveBaseGoal()
	
	#set up the frame parameters
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	# moving towards the goal
	goal.target_pose.pose.position =  Point(goalx,goaly,0)
	#convert the roll-pitch-yaw angles to a quaternion using ROS TF Library
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, goalyaw)
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]

	rospy.loginfo("Sending goal location ...")
	ac.send_goal(goal)

	ac.wait_for_result(rospy.Duration(60))
	
	if(ac.get_state() ==  GoalStatus.SUCCEEDED):
		rospy.loginfo("robot reached goal pose")
		return True
	else:
		rospy.loginfo("The robot failed to reach goal pose")
		return False

def getgoal(csvfilepath):
	with open(csvfilepath,'r') as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		csv_reader = list(csv_reader)
	return float(csv_reader[1][1]),float(csv_reader[1][2]),math.radians(float(csv_reader[1][3]))

if __name__ == '__main__':
	rospy.init_node('map_navigation', anonymous=False)
	filepath = sys.argv[1]
	goalx, goaly, goalyaw = getgoal(filepath)
	print('start go to goal')
	move_to_goal(goalx,goaly,goalyaw)
	rospy.spin()
	