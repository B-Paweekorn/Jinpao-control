#include "Kinematics.h"

Kinematics::Kinematics(float _wheelDiameter, float _lx, float _ly) {
    robot.wheelDiameter = _wheelDiameter;
    robot.lx = _lx;
    robot.ly = _ly;
    robot.wheelCircumference = robot.wheelDiameter * M_PI;
    robot.angular_to_rpm = 60 / robot.wheelCircumference;

    
}