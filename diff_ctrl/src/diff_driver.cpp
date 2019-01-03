#include <cmath>
#include "diff_driver.h"
#include "l298n.h"

DifferentialDriver::DifferentialDriver()
{
	lrw_cmd = new double[2];
	lrw_cmd[WHEEL_LEFT] = 0.0;
	lrw_cmd[WHEEL_RIGHT] = 0.0;
	ros::NodeHandle nh("~");
	nh.param("logical_voltage", 	v_logic, 1.0);
	ROS_INFO("logical voltage set at %f", v_logic);
	cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>
	(
		"/cmd_vel",
		10,
		&DifferentialDriver::driverCallback,
		this
	);
	motor_driver.initialise();
}

double* DifferentialDriver::getLR() const
{
	return lrw_cmd;
}

void DifferentialDriver::driverCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	rawToWheel(cmd);
}

void DifferentialDriver::rawToWheel(const geometry_msgs::Twist::ConstPtr& cmd)
{
	double fwd = cmd -> linear.x;
	double yaw = cmd -> angular.z;
	double tot = fabs(fwd) + fabs(yaw);

	if ( tot < 0.05 )
	{
		lrw_cmd[WHEEL_LEFT] = 0.0;
		lrw_cmd[WHEEL_RIGHT] = 0.0;
	}else
	{
		lrw_cmd[WHEEL_LEFT] = fabs(fwd) * fwd/tot - fabs(yaw) * yaw/tot;
		lrw_cmd[WHEEL_RIGHT] = fabs(fwd) * fwd/tot + fabs(yaw) * yaw/tot;
	}
	motor_driver.convertCommand(lrw_cmd[WHEEL_LEFT], WHEEL_LEFT);
	ROS_INFO( "left wheel %f", v_logic * lrw_cmd[WHEEL_LEFT]);

	motor_driver.convertCommand(lrw_cmd[WHEEL_RIGHT], WHEEL_RIGHT);
	ROS_INFO( "right wheel %f", v_logic * lrw_cmd[WHEEL_RIGHT]);

	motor_driver.executeCommand();
}
