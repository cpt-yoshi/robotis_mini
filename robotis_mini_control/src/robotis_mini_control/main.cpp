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
	ros::init(argc, argv, "Darwin_Joint_Control");

	/**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle node;
	DarwinJointControl darwin(&node);
	DarwinReadFile table("/home/biobot/Desktop/data.txt");
	

	// Valeurs initiales des angles du robot
	const double hipOffsetY = .024; //OP, measured
	const double hipOffsetZ = .027; //OP, Calculated from spec
	const double hipOffsetX = .015; //OP, Calculated from spec
	const double thighLength = .045; //OP, spec
	const double tibiaLength = .042; //OP, spec
	const double footHeight = .0246; //OP, spec
	const double kneeOffsetX = 0.04; //This parameter can be modified
	//const double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
	const double dThigh = thighLength;
	//const double aThigh = atan(kneeOffsetX/thighLength);
const double aThigh = 0;
	//const double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
	const double dTibia = tibiaLength;
	const double aTibia = atan(kneeOffsetX/tibiaLength);
	

	ROS_INFO("dThigh = %f", dThigh);
	ROS_INFO("aThigh = %f", aThigh);
	ROS_INFO("dTibia = %f", dTibia);
	ROS_INFO("aTibia = %f", aTibia);

	double N = table.getN(0, 0);	//Nombre d'itérations à faire 
	double dt = table.getTime(1, 0);	//Nombre d'itérations à faire 

	double z0=0;
	double z1=0;
	double z2=0;
	double z3=0;
	double z4=0;
	double z5=0;

	ros::Duration time_tot(15); 
	ros::Duration test(5);
	//ros::Duration temps(0.5);
ros::Duration temps(dt);
	ros::Rate rate(50);
	
	const double a_hip_i = 0.45;
	const double a_foot_i = 0.45;

	ROS_INFO("r_motor_hip = %f", a_hip_i);
	ROS_INFO("r_motor_thigh = %f", -aThigh);
	ROS_INFO("r_motor_knee = %f", aThigh+aTibia);
	ROS_INFO("r_motor_ankle = %f", -aTibia);
	ROS_INFO("r_motor_foot = %f", a_foot_i);



	// Placer a zero
	darwin.goalPos("l_hip_joint", 	0, 		  time_tot, rate);
	darwin.goalPos("r_hip_joint", 	0, 		  time_tot, rate);
	darwin.goalPos("l_thigh_joint", 0, 	  time_tot, rate);
	darwin.goalPos("r_thigh_joint", 0, 	  time_tot, rate);
	darwin.goalPos("l_knee_joint", 	0, 	  time_tot, rate);
	darwin.goalPos("r_knee_joint", 	0, time_tot, rate);
	darwin.goalPos("l_ankle_joint", 0, 	  time_tot, rate);
	darwin.goalPos("r_ankle_joint", 0, 	  time_tot, rate);
	darwin.goalPos("l_foot_joint",	0, 		  time_tot, rate);
	darwin.goalPos("r_foot_joint", 	0, 		  time_tot, rate);


	darwin.goalPos("l_biceps_joint", 	0, 		  time_tot, rate);
	darwin.goalPos("r_biceps_joint", 	0, 		  time_tot, rate);
	darwin.goalPos("r_shoulder_joint", 	0, 		  time_tot, rate);
	darwin.goalPos("l_shoulder_joint", 	0, 		  time_tot, rate);
	darwin.goalPos("l_elbow_joint", 	0, 		  time_tot, rate);
	darwin.goalPos("r_elbow_joint", 	0, 		  time_tot, rate);
	time_tot.sleep();

	//Place le robot à la position initiale permettant son pas
	darwin.goalPos("l_hip_joint", 	-a_hip_i, 		  time_tot, rate);
	darwin.goalPos("r_hip_joint", 	a_hip_i, 		  time_tot, rate);
	darwin.goalPos("l_thigh_joint", -aThigh, 	  time_tot, rate);
	darwin.goalPos("r_thigh_joint", -aThigh, 	  time_tot, rate);
	darwin.goalPos("l_knee_joint", 	(aThigh+aTibia), 	  time_tot, rate);
	darwin.goalPos("r_knee_joint", 	aThigh+aTibia, time_tot, rate);
	darwin.goalPos("l_ankle_joint", -aTibia, 	  time_tot, rate);
	darwin.goalPos("r_ankle_joint", -aTibia, 	  time_tot, rate);
	darwin.goalPos("l_foot_joint",	-a_foot_i, 		  time_tot, rate);
	darwin.goalPos("r_foot_joint", 	a_foot_i, 		  time_tot, rate);

	darwin.goalPos("r_biceps_joint", 	1.1, 		  time_tot, rate);
	//darwin.goalPos("l_biceps_joint", 	1.5, 		  time_tot, rate);

	time_tot.sleep();

	test.sleep();
	
	//return 0;

	//Boucle permettant le pas 
 
	int n=0;
	while (n < N)
		{
			z0=table.getJoint(n+2, 1);
			z1=table.getJoint(n+2, 2);
			z2=table.getJoint(n+2, 3);
			z3=table.getJoint(n+2, 4);
			z4=table.getJoint(n+2, 5);
			//z5=table.getJoint(n,5);
			
			n += 1;
			//darwin.setJoint("j_pelvis_r", z0);
			darwin.setJoint("r_hip_joint", z0);
			darwin.setJoint("r_thigh_joint", z1);
			darwin.setJoint("r_knee_joint", z2);
			darwin.setJoint("r_ankle_joint", z3);
			darwin.setJoint("r_foot_joint", z4);
			//darwin.setJoint("r_biceps_joint", -1.5);
			temps.sleep();
		}
		
	
	return 0;


}
