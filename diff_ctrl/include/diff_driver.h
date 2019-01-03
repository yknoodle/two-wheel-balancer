#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "l298n.h"
class DifferentialDriver
{
public:

	DifferentialDriver();

	// get pointer to reference wheel velocity signal
	double* getLR() const;
	void driverCallback( const geometry_msgs::Twist::ConstPtr& );
	void rawToWheel(const geometry_msgs::Twist::ConstPtr& cmd);
	
private:

	enum Wheel
	{
		WHEEL_LEFT,
		WHEEL_RIGHT,
		WHEEL_NONE
	};

	double v_logic;
	double* lrw_cmd;

	L298NDriver motor_driver;
	ros::NodeHandle nh_;
	ros::Subscriber cmd_sub_;
};
