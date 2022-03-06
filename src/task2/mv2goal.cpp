#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void getgoal(std::string fname, float *goalx, float *goaly, float *goalyaw){ 
	std::vector<std::vector<std::string>> content;
	std::vector<std::string> row;
	std::string line, word; 
	std::fstream file; //(fname, std::ios::in);
  file.open(fname, std::fstream::in);
	if(file.is_open()){
		while(std::getline(file, line)){
			row.clear();
			std::stringstream str(line); 
			while(std::getline(str, word, ','))
				row.push_back(word);
			content.push_back(row);
		}
	}
	else
		std::cout<<"Could not open the file\n";
 
	for(int i=0;i<content.size();i++){
		for(int j=0;j<content[i].size();j++){
			std::cout<<content[i][j]<<" ";
		}
		std::cout<<"\n";
	}
  *goalx=std::stof(content[1][1]);
  *goaly=std::stof(content[1][2]);
  *goalyaw=std::stof(content[1][3])*M_PI/180; //convert degrees to radians
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  std::string fname = argv[1];
  float goalx, goaly, goalyaw;
  getgoal(fname, &goalx, &goaly, &goalyaw);
  printf("goalx: %f\ngoaly: %f\ngoalyaw: %f\n",goalx,goaly,goalyaw);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //build move_base goal message
  move_base_msgs::MoveBaseGoal goal;
 
  goal.target_pose.header.frame_id = "map"; //goal coordinates relative to map
  goal.target_pose.header.stamp = ros::Time::now();
  
  // convert the roll-pitch-yaw angles to a quaternion using ROS TF2 Library
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, goalyaw); // Create this quaternion from roll/pitch/yaw (in radians)

  goal.target_pose.pose.position.x = goalx;
  goal.target_pose.pose.position.y = goaly;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = quaternion[2];
  goal.target_pose.pose.orientation.w = quaternion[3];

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached");
  else
    ROS_INFO("Failed to reach goal");

  return 0;
}