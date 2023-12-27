#ifndef PID_H
#define PID_H

#include "setMotor.h"
#include "QEI.h"
#include <math.h>

class PID {
private:
  setMotor* motor;  // Pointer to a setMotor object
  QEI* enc;
  float dt = 1 / 1000.0;
  float kp, ki, kd;
  float eintegral = 0;
  float v = 0;
  double u = 0;
  double u_prev = 0;
  float e_prev = 0;
  float e_prev2 = 0;
  int32_t pos_prev = 0;
  float setRads = 0;
  float counts_per_rev = 4000.0;
  float a = 45;

public:
  PID(setMotor* _motor, QEI* _enc, float _kp, float _ki, float _kd);  // Constructor that takes a setMotor object
  void compute();
  void setRads(float _setRads);
  void setK(float _kp, float _ki, float _kd);
};

#endif
