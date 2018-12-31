/*

Code for GEI744, ROS and Darwin_Op robotic plateform.
Author: Guillaume Durandau, Guillaume.Durandau@Usherbrooke.ca
License: BSD

Vers: 0.0.1
Date: 05/02/14

*/

#include "ros/ros.h"
#include "lib/DarwinJointControl.h"
#include "lib/DarwinReadFile.h"
/*
	Joint : j_pelvis_r, j_thigh1_r, j_thigh2_r, j_tibia_r, j_ankle1_r, j_ankle2_r
*/


int main(int argc, char **argv)
{
	/**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
   	if(argc < 5)
   	{
   		ROS_ERROR("Usage: name_of_the_joint joint_value time rate");
   		return 0;
   	}
	ros::init(argc, argv, "Darwin_Joint_Control_commande");

	/**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle node;
	DarwinJointControl darwin(&node);
	
	ros::Duration time_tot(atof(argv[3]));
	ros::Rate rate(atof(argv[4]));
	//*
	darwin.goalPos(argv[1], atof(argv[2]), time_tot, rate);

	//*/
	time_tot.sleep();
	
	return 0;
}
