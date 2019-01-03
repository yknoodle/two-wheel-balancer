#include "diff_driver.h"

int main(int argc, char** argv)
{
	ROS_INFO("Differential driver started.");
	ros::init(argc, argv, "driver");

	DifferentialDriver driver;

	ros::spin();
}
