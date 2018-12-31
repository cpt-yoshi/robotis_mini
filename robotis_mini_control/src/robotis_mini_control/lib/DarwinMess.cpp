#include "DarwinMess.h"

DarwinMess::DarwinMess(NodeHandle& node)
{
	std::ostringstream joint_name_state_;
	XmlRpc::XmlRpcValue list_joint_name;
	Rate rate(50);
	if(node.getParam("/robotis_mini/joint_list", list_joint_name))
	{
		ROS_ASSERT(list_joint_name.getType() == XmlRpc::XmlRpcValue::TypeArray);
		for(int i = 0; i < list_joint_name.size(); ++i)
		{
			joint_name_state_<< "/robotis_mini/" << static_cast<std::string>(list_joint_name[i]).c_str() << "_position_controller/state";
			pub_in_[static_cast<std::string>(list_joint_name[i])] = node.advertise<control_msgs::JointControllerState>(joint_name_state_.str().c_str(), 10);
			sub_out_[static_cast<std::string>(list_joint_name[i])] = new DarwinCallback(&node, static_cast<std::string>(list_joint_name[i]));
			//callback declared but never call /!/
			joint_name_state_.str("");
			joint_name_state_.clear();
		}
	}
	pub_out_ = node.advertise<sensor_msgs::JointState>("/command_joint_states", 1000);
	sub_in_ = node.subscribe("/states_joint_states", 1000, &DarwinMess::callback_In, this);
	thread_pub_ = new boost::thread(&DarwinMess::pub_out, this, pub_out_, rate);
}

void DarwinMess::callback_In(const sensor_msgs::JointState::ConstPtr& msg_in)
{
	control_msgs::JointControllerState msg_out;
	vector<string>::const_iterator it;
	vector<double>::const_iterator it_p = msg_in->position.begin();
	for(it = msg_in->name.begin(); it != msg_in->name.end(); ++it)
	{
		msg_out.header = msg_in->header;
		msg_out.set_point = (*it_p);
		pub_in_[(*it)].publish(msg_out);
		++it_p;
	}
}

void DarwinMess::pub_out(Publisher pub_out, Rate rate)
{
	sensor_msgs::JointState msg_out;
	map<string, double> temp;
	while(ros::ok())
	{
		DarwinCallback::msg_data.mutex_.lock();
		temp = DarwinCallback::msg_data.data;
		DarwinCallback::msg_data.mutex_.unlock();
		msg_out.header.stamp = ros::Time::now();
		for(map<string, double>::iterator it = temp.begin(); it != temp.end(); ++it)
		{
			msg_out.name.push_back(it->first);
			msg_out.position.push_back(it->second);
		}
		pub_out.publish(msg_out);
		rate.sleep();
		msg_out.name.clear();
		msg_out.position.clear();
	}
}











