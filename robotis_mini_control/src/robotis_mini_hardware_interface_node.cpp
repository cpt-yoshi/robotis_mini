#include <robotis_mini_hardware_interface/robotis_mini_hardware_interface.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotis_mini_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    robotis_mini_hardware_interface::RobotisMiniHardwareInterface ROBOT(nh);
    ros::spin();
    return 0;
}
