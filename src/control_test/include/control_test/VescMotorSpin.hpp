#ifndef VESC_MOTOR_SPIN_HPP
#define VESC_MOTOR_SPIN_HPP

#include <string>
#include <ros/ros.h>
#include "can_wrapper/Wheels.h"
#include "can_wrapper/RosCanConstants.hpp"

class VescMotorSpin
{
public:
	VescMotorSpin(const ros::NodeHandle &nh, const ros::Rate &loopRate);

	void stop();
	void start();
	void setVelocity(const can_wrapper::Wheels &msg);
	void setLoopRate(const ros::Rate &loopRate);

private:
	ros::NodeHandle mNh;
	ros::Publisher mMotorVelPub;
	can_wrapper::Wheels mSetVel;
	ros::Timer mTimerLoc;



	void handleTimerCallback(const ros::TimerEvent &);
};

#endif // VESC_MOTOR_SPIN_HPP
