#include "ros/ros.h"
#include "lib/DarwinMess.h"

using namespace ros;

int main(int argc,char** argv)
{
	ros::init(argc, argv, "nodeTrans");
	ros::NodeHandle node;
	DarwinMess darwin_t(node);
	
	ros::spin();

  	return 0;
}
