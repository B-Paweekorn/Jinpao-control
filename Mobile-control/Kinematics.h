#include <math.h>
class Kinematics {
private:

  //Robot
  struct Robot {
    float wheelDiameter;
    float lx;
    float ly;
    float wheelCircumference;
    float angular_to_rpm;
  };

  Robot robot;  // Adding Robot object as a member

  //RadPS of each wheel for inverse kinematic
  struct RadPS {
    float RadPS_FL;
    float RadPS_FR;
    float RadPS_BL;
    float RadPS_BR;
  };

  //Velocity for forward kinematic
  struct Velocity {
    float vx;
    float vy;
    float wz;
  };

  //Postion
  struct Position {
    float x;
    float y;
    float theta;
  };

public:

  RadPS Inverse_Kinematics(float vx, float vy, float wz);
  Velocity Forward_Kinematics(float rpm_FL, float rpm_FR, float rpm_BL, float rpm_BR);
  Kinematics(float _wheelDiameter, float _lx, float _ly);
};