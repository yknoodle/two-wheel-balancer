#include "binder.h"

XB360Joy::XB360Joy():
	a_scale_(1),
	l_scale_(1),
	c_name_("")
{
	// store param value to axis_bind if available.
	nh_.param("cmd_axis_tx", 	axis_bind[CMD_AXIS_TX],(int)	JOY_AXIS_LY);
	nh_.param("cmd_axis_ty", 	axis_bind[CMD_AXIS_TY],(int)	JOY_AXIS_NONE);
	nh_.param("cmd_axis_tz", 	axis_bind[CMD_AXIS_TZ],(int)	JOY_AXIS_NONE);

	nh_.param("cmd_axis_roll", 	axis_bind[CMD_AXIS_ROLL],(int)	JOY_AXIS_NONE);
	nh_.param("cmd_axis_pitch",	axis_bind[CMD_AXIS_PITCH],(int)	JOY_AXIS_NONE);
	nh_.param("cmd_axis_yaw", 	axis_bind[CMD_AXIS_YAW],(int)	JOY_AXIS_RX);

	nh_.param("scale_angular", 	a_scale_, 	a_scale_);
	nh_.param("scale_linear", 	l_scale_, 	l_scale_);
	nh_.param("controller", 	c_name_, 	c_name_);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>
	(
		c_name_ + "/cmd_vel",
		1
	);


	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>
	(
		"joy",
		10,
		&XB360Joy::joyCallback, 
		this
	);

}

void XB360Joy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	getJoyAxes(joy);
	update();
	vel_pub_.publish(twist);
}

void XB360Joy::update()
{
	// TODO: update the bindings as well.

	// update the twist message with axes input
	twist.linear.x = axis_joy[ axis_bind[ CMD_AXIS_TX ] ];
	twist.linear.y = axis_joy[ axis_bind[ CMD_AXIS_TY ] ];
	twist.linear.z = axis_joy[ axis_bind[ CMD_AXIS_TZ ] ];
	twist.angular.x = axis_joy[ axis_bind[ CMD_AXIS_ROLL] ];
	twist.angular.y = axis_joy[ axis_bind[ CMD_AXIS_PITCH] ];
	twist.angular.z = axis_joy[ axis_bind[ CMD_AXIS_YAW] ];
}

void XB360Joy::getJoyAxes(const sensor_msgs::Joy::ConstPtr& joy)
{
	// get axes input from joy message
	double LX_val = joy -> axes[JOY_AXIS_LX];
	double LY_val = joy -> axes[JOY_AXIS_LY];
	double LT_val = joy -> axes[JOY_AXIS_LT];

	double RX_val = joy -> axes[JOY_AXIS_RX];
	double RY_val = joy -> axes[JOY_AXIS_RY];
	double RT_val = joy -> axes[JOY_AXIS_RT];

	axis_joy[JOY_AXIS_LX] = LX_val;
	axis_joy[JOY_AXIS_LY] = LY_val;
	axis_joy[JOY_AXIS_LT] = -(LT_val - 1);

	axis_joy[JOY_AXIS_RX] = RX_val;
	axis_joy[JOY_AXIS_RY] = RY_val;
	axis_joy[JOY_AXIS_RT] = -(RT_val - 1);

}
