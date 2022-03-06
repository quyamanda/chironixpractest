/*

 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <string>
#include <chrono>

int main(int argc, char **argv)
{
    // Initiate new ROS node named "talker"
	ros::init(argc, argv, "cw");

	//create node handle
	ros::NodeHandle n;
	//create publisher
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	//Define frequency for loop
	ros::Rate loop_rate(50); //Hz

    auto t0 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tdiff = std::chrono::high_resolution_clock::now()-t0;
   while (tdiff.count() < 10) // Keep spinning loop until 10s or user presses Ctrl+C
   {
       //create new ROS message.
	   geometry_msgs::Twist msg;

        // set values for cmd_vel message
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = -std::stof(argv[1]);
       //print the content of the message in the terminal
    //    ROS_INFO("[Talker] I published %s\n", msg.data.c_str());
        // ROS_INFO(msg);
       //Publish the message
       pub.publish(msg);
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

      loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
       tdiff = std::chrono::high_resolution_clock::now() - t0;
   }
   return 0;
}


