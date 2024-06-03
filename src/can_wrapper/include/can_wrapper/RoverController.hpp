#ifndef ROVER_CONTROLLER_HPP
#define ROVER_CONTROLLER_HPP

#include <ros/ros.h>
#include <can_wrapper/RoverControl.h>
#include <can_wrapper/Wheels.h>

class RoverController
{
public:
    RoverController(double length, double width);
    void roverControlCallback(const can_wrapper::RoverControl::ConstPtr &msg);
    can_wrapper::Wheels prepareWheelsMessage() const;

private:
    double XVelAxis;
    double ZRotAxis;
    double L; // Length between front and back wheels
    double W; // Width between left and right wheels
};


double tangent360(double y, double x);


#endif // ROVER_CONTROLLER_HPP
