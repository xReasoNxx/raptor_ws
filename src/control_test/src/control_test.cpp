#include <ros/ros.h>
#include <can_wrapper/Wheels.h>
#include "control_test/VescMotorSpin.hpp"

ros::Time startTime = ros::Time::now();

void handleTimerCallback(const ros::TimerEvent &)
{
	can_wrapper::Wheels vel;
	vel.frontRight = 0.0f;
	vel.midRight = 0.0f;
	vel.rearRight = 0.0f;
	vel.frontLeft = 0.0f;
	vel.midLeft = 0.0f;
	vel.rearLeft = 0.0f;

	if (ros::Time::now() - startTime < ros::Duration(5))
	{
		
	}
	else if (ros::Time::now() - startTime < ros::Duration(15))
	{
		vel.frontRight = 0.0f;
		vel.midRight = 1.0f;
		vel.rearRight = 0.0f;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "control_test");
	ros::NodeHandle n;

	VescMotorSpin motorSpin(n, ros::Rate(50));

	


	ros::spin();

	motorSpin.stop();
	return 0;
}
