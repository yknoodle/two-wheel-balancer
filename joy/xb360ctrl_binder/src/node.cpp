#include "binder.h"

int main(int argc, char** argv)
{
	ROS_INFO("XBox 360 controller binder started");
	ros::init(argc, argv, "binder");
	XB360Joy joy;
	ros::spin();
}
