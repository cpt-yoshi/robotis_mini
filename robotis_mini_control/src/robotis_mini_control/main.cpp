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
	Joint : r_shoulder_joint, l_shoulder_joint, r_biceps_joint, l_biceps_joint, r_elbow_joint, l_elbow_joint, 
          r_hip_joint, l_hip_joint, r_thigh_joint, l_thigh_joint, r_knee_joint, l_knee_joint, 
          r_ankle_joint, l_ankle_joint, r_foot_joint, l_foot_joint
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


  /* Example */
  /*
  // Ouverture du fichier contenant les joints
  DarwinReadFile file("/home/biobot/Desktop/data.txt");

  double N  = file.getTSV(0, 0);  //Nombre d'itérations à faire
  double dt = file.getTSV(1, 0);  //Periode d'échantillonage
  double freq = 1 / dt;

  // Dimension du robot
  const double Lx_hip = 0.015;
  const double Ly_hip = 0.024;
  ...

  // Joints initiales
  const double theta1_i = -0.45;
  const double theta2_i = 0;
  ...


  ros::Duration tf(10); 
  ros::Rate rate(freq);

  // Place le robot a son etat initial (voir guide)
  darwin.goalPos("r_hip_joint",    theta1_i, tf, rate);
  darwin.goalPos("l_hip_joint",    theta1_i, tf, rate);
  darwin.goalPos("r_thigh_joint",  theta2_i, tf, rate);
  darwin.goalPos("l_thigh_joint", -theta2_i, tf, rate);
  darwin.goalPos("r_knee_joint",   theta3_i, tf, rate);
  darwin.goalPos("l_knee_joint",  -theta3_i, tf, rate);
  darwin.goalPos("r_ankle_joint",  theta4_i, tf, rate);
  darwin.goalPos("l_ankle_joint", -theta4_i, tf, rate);
  darwin.goalPos("r_foot_joint",   theta5_i, tf, rate);
  darwin.goalPos("l_foot_joint",   theta5_i, tf, rate);

  // Pour faciliter la lever du pied droit
  darwin.goalPos("r_biceps_joint", 1.1,      tf, rate);

  tf.sleep();

  // Faire la trajectoire issue de votre fichier (generer par MatLab)
  ...

  */


  return 0;


}
