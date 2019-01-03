#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class XB360Joy
{
public:
  XB360Joy();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void getJoyAxes(const sensor_msgs::Joy::ConstPtr& joy);
	void update();

	ros::NodeHandle nh_;

	enum CommandAxis
	{
		CMD_AXIS_TX,
		CMD_AXIS_TY,
		CMD_AXIS_TZ,
		CMD_AXIS_ROLL,
		CMD_AXIS_PITCH,
		CMD_AXIS_YAW,
		CMD_AXIS_NONE
	};
	
	enum JoyAxis
	{
		JOY_AXIS_LX,
		JOY_AXIS_LY,
		JOY_AXIS_LT,
		JOY_AXIS_RX,
		JOY_AXIS_RY,
		JOY_AXIS_RT,
		JOY_AXIS_HX,
		JOY_AXIS_HY,
		JOY_AXIS_NONE
	};
	int axis_bind[6] = 
	{
		JOY_AXIS_NONE, 
		JOY_AXIS_NONE,
		JOY_AXIS_NONE,
		JOY_AXIS_NONE,
		JOY_AXIS_NONE,
		JOY_AXIS_NONE
	};

	geometry_msgs::Twist twist;

	double axis_joy[9]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	double	l_scale_,
			a_scale_;
	std::string c_name_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
};

