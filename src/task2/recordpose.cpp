#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <string>
#include <chrono>
#include <iostream>
#include <ctime>
#include <fstream>
std::fstream csvfile;

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
  ros::init(argc, argv, "recordpose");
  ros::NodeHandle node;
  std::string filepath = argv[1];
  // std::string csvline = getTimeStr()+","+std::to_string(curr_x);
  csvfile.open(filepath, std::ios_base::app | std::ios_base::in);
  ros::Subscriber sub = node.subscribe("amcl_pose", 1000, callback_pose);
  ros::spin();
  // csvfile.close();
  return 0;
}