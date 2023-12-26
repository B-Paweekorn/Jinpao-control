#include "Kinematics.h"

Kinematics::Kinematics(float _wheelDiameter, float _lx, float _ly) {
  robot.wheelDiameter = _wheelDiameter;
  robot.lx = _lx;
  robot.ly = _ly;
  robot.wheelCircumference = robot.wheelDiameter * M_PI;
  robot.angular_to_rpm = 60 / robot.wheelCircumference;
}

Kinematics::Velocity Kinematics::Forward_Kinematics(float RadPS_FL, float RadPS_FR, float RadPS_BL, float RadPS_BR) {
  Velocity basev;
  basev.vx = (robot.wheelDiameter / 2) / 4 * (RadPS_FL + RadPS_FR + RadPS_BL + RadPS_BR);
  basev.vy = (robot.wheelDiameter / 2) / 4 * (-RadPS_FL + RadPS_FR + RadPS_BL - RadPS_BR);
  basev.wz = (robot.wheelDiameter / 2) / (4 * (robot.lx + robot.ly)) * (RadPS_FL - RadPS_FR + RadPS_BL - RadPS_BR);
  return basev;
}

Kinematics::RadPS Kinematics::Inverse_Kinematics(float vx, float vy, float wz) {
  RadPS wheel_rads;
  wheel_rads.RadPS_FL = (vx + vy + (robot.lx + robot.ly) * wz);
  wheel_rads.RadPS_FR = (vx - vy - (robot.lx + robot.ly) * wz);
  wheel_rads.RadPS_BL = (vx - vy + (robot.lx + robot.ly) * wz);
  wheel_rads.RadPS_BR = (vx + vy - (robot.lx + robot.ly) * wz);
  return wheel_rads;
}
