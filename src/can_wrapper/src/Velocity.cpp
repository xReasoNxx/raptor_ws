#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>
#include <cmath>

#include "can_wrapper/RoverControl.h"
#include "can_wrapper/Wheels.h"
#include "can_wrapper/Wheel.h"
#include "can_wrapper/MotorControl.hpp"

double targetLinearVelocity;
double targetAngularVelocity;
int IdAngle = 4;
int Id = 0;
double Vx;
double Vy;
double omega;
const double L = 0.5; // Wheelbase (distance between front and rear wheels)
const double W = 0.3; // Track width (distance between left and right wheels)

static void roverControlCallback(const can_wrapper::RoverControl::ConstPtr &msg)
{
	targetLinearVelocity = msg->XVelAxis;
	targetAngularVelocity = msg->ZRotAxis;
	

}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;
	MotorControl motorControl(n);

	ros::Rate rate(1000);
	ros::Subscriber sub = n.subscribe("/MQTT/RoverControl", 100, roverControlCallback);
	can_wrapper::Wheels vel;
	while (ros::ok())
    {
		can_wrapper::Wheels vel;
        // Calculate velocity and angle for each wheel based on target velocities and wheel geometry
        // Calculate velocities for each wheel
		double V_FL_x = Vx - omega * W / 2;
		double V_FL_y = Vy + omega * L / 2;
		double V_FR_x = Vx + omega * W / 2;
		double V_FR_y = Vy + omega * L / 2;
		double V_RL_x = Vx - omega * W / 2;
		double V_RL_y = Vy - omega * L / 2;
		double V_RR_x = Vx + omega * W / 2;
		double V_RR_y = Vy - omega * L / 2;

		// Calculate resultant velocities
		double V_FL = sqrt(V_FL_x * V_FL_x + V_FL_y * V_FL_y);
		double V_FR = sqrt(V_FR_x * V_FR_x + V_FR_y * V_FR_y);
		double V_RL = sqrt(V_RL_x * V_RL_x + V_RL_y * V_RL_y);
		double V_RR = sqrt(V_RR_x * V_RR_x + V_RR_y * V_RR_y);

		// Calculate angles in radians
		double theta_FL = atan2(V_FL_y, V_FL_x);
		double theta_FR = atan2(V_FR_y, V_FR_x);
		double theta_RL = atan2(V_RL_y, V_RL_x);
		double theta_RR = atan2(V_RR_y, V_RR_x);

		// Convert angles to degrees
		theta_FL *= 180 / M_PI;
		theta_FR *= 180 / M_PI;
		theta_RL *= 180 / M_PI;
		theta_RR *= 180 / M_PI;

		// Prepare the Wheels message
		vel.header.stamp = ros::Time::now();

		vel.frontLeft.setValue = V_FL;
		vel.frontLeft.setAngle = theta_FL;
		vel.frontLeft.commandIdAngle = IdAngle;
		vel.frontLeft.commandId = Id;

		vel.frontRight.setValue = V_FR;
		vel.frontRight.setAngle = theta_FR;
		vel.frontRight.commandIdAngle = IdAngle;
		vel.frontRight.commandId = Id;

		vel.rearLeft.setValue = V_RL;
		vel.rearLeft.setAngle = theta_RL;
		vel.rearLeft.commandIdAngle = IdAngle;
		vel.rearLeft.commandId = Id;

		vel.rearRight.setValue = V_RR;
		vel.rearRight.setAngle = theta_RR;
		vel.rearRight.commandIdAngle = IdAngle;
		vel.rearRight.commandId = Id;

		motorControl.sendMotorVel(vel);
        ros::spinOnce();
        rate.sleep();
    }
}
