#include <cmath> 
#include <ros/ros.h>
#include "can_wrapper/RoverController.hpp"

double tangent360(double y, double x)
{
    // Calculate the angle in radians using atan2
    double angle_rad = atan2(y, x);

    // Convert angle to degrees
    double angle_deg = angle_rad * 180.0 / M_PI;

    // Ensure angle is in the range [0, 360)
    angle_deg = fmod(angle_deg, 360.0);
    if (angle_deg < 0) {
        angle_deg += 360.0;
    }
    return angle_deg;
}


RoverController::RoverController(double length, double width)
    : XVelAxis(0), ZRotAxis(0), L(length), W(width) {}

void RoverController::roverControlCallback(const can_wrapper::RoverControl::ConstPtr &msg)
{
    XVelAxis = msg->XVelAxis;
    ZRotAxis = msg->ZRotAxis;
}

can_wrapper::Wheels RoverController::prepareWheelsMessage() const
{
    can_wrapper::Wheels vel;

    // Calculate velocity and angle for each wheel based on target velocities and wheel geometry
    // Calculate velocities for each wheel
    double omega = ZRotAxis / L;
    double V_FL_x = XVelAxis - omega * W / 2;
    double V_FL_y = ZRotAxis + omega * L / 2;
    double V_FR_x = XVelAxis + omega * W / 2;
    double V_FR_y = ZRotAxis + omega * L / 2;
    double V_RL_x = XVelAxis - omega * W / 2;
    double V_RL_y = ZRotAxis - omega * L / 2;
    double V_RR_x = XVelAxis + omega * W / 2;
    double V_RR_y = ZRotAxis - omega * L / 2;

    // Calculate resultant velocities
    double V_FL = sqrt(V_FL_x * V_FL_x + V_FL_y * V_FL_y);
    double V_FR = sqrt(V_FR_x * V_FR_x + V_FR_y * V_FR_y);
    double V_RL = sqrt(V_RL_x * V_RL_x + V_RL_y * V_RL_y);
    double V_RR = sqrt(V_RR_x * V_RR_x + V_RR_y * V_RR_y);

    // Calculate angles in radians
    double theta_FL = tangent360(V_FL_y, V_FL_x);
    double theta_FR = tangent360(V_FR_y, V_FR_x);
    double theta_RL = tangent360(V_RL_y, V_RL_x);
    double theta_RR = tangent360(V_RR_y, V_RR_x);

    // Prepare the Wheels message
    int IdAngle = 4;
    int Id = 0;

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
    return vel;
}
