#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::fstream csvfile;

void getgoal(std::string fname, float *goalx, float *goaly, float *goalyaw){ 
  std::vector<std::vector<std::string>> content;
  std::vector<std::string> row;
  std::string line, word; 
  std::fstream file;
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
 
  *goalx=std::stof(content[1][1]);
  *goaly=std::stof(content[1][2]);
  *goalyaw=std::stof(content[1][3])*M_PI/180; //convert degrees to radians
}

std::string getCurrTimeStr(){
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return s;
}

void callback_pose(const geometry_msgs::PoseWithCovarianceStamped msg){
  float curr_x,curr_y;
  double curr_roll,curr_pitch,curr_yaw;
  curr_x = msg.pose.pose.position.x;
  curr_y = msg.pose.pose.position.y;
  //formulate a quaternion as a list
  geometry_msgs::Quaternion quat_msg;
  quat_msg = msg.pose.pose.orientation;
  tf2::Quaternion quat_tf;
  tf2::convert(quat_msg , quat_tf);
  tf2::Matrix3x3 rpy(quat_tf);
  //convert the quaternion to roll-pitch-yaw
  rpy.getRPY(curr_roll, curr_pitch, curr_yaw);
  curr_yaw = curr_yaw*180/M_PI; //convert radians to degrees
  if (csvfile.is_open())
    csvfile << getCurrTimeStr()+','+std::to_string(curr_x)+','+std::to_string(curr_y)+','+std::to_string(curr_yaw) << std::endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move2goal");
  ros::NodeHandle node;
  std::string fname = argv[1];
  float goalx, goaly, goalyaw;
  getgoal(fname, &goalx, &goaly, &goalyaw);
  printf("goalx: %f\ngoaly: %f\ngoalyaw: %f\n",goalx,goaly,goalyaw);
  std::string filepath = argv[1];
  csvfile.open(filepath, std::ios_base::app | std::ios_base::in);
  ros::Subscriber sub = node.subscribe("amcl_pose", 1000, callback_pose);
  
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

  ros::spin();
  
  // ac.waitForResult();

  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("Goal reached");
  //   csvfile.close();
  // else
  //   ROS_INFO("Failed to reach goal");

  return 0;
}
