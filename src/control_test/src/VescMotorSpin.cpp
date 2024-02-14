#include "control_test/VescMotorSpin.hpp"

VescMotorSpin::VescMotorSpin(const ros::NodeHandle &nh, const ros::Rate &loopRate)
	: mNh(nh)
{
	// should be RosCanConstants::RosTopics::can_set_motor_vel but it wont link
	mMotorVelPub = mNh.advertise<can_wrapper::Wheels>("/CAN/TX/set_motor_vel", 64);
	mTimerLoc = mNh.createTimer(loopRate.expectedCycleTime(), &VescMotorSpin::handleTimerCallback, this, false, false);
}

void VescMotorSpin::stop()
{
	mTimerLoc.stop();
}

void VescMotorSpin::start()
{
	mTimerLoc.start();
}

void VescMotorSpin::setVelocity(const can_wrapper::Wheels &msg)
{
	mSetVel = msg;
}

void VescMotorSpin::setLoopRate(const ros::Rate &loopRate)
{
	mTimerLoc.setPeriod(loopRate.expectedCycleTime());
}

void VescMotorSpin::handleTimerCallback(const ros::TimerEvent &)
{
	mMotorVelPub.publish(mSetVel);
}
