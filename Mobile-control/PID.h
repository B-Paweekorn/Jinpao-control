#ifndef PID_H
#define PID_H

#include "setMotor.h"
#include "QEI.h"
#include <math.h>
#include "KalmanFilter.h"

class PID {
private:


public:

  setMotor* motor;  // Pointer to a setMotor object
  QEI* enc;
  KalmanFilter* kf;
  float dt = 1 / 1000.0;
  float kp, ki, kd;
  float eintegral = 0;
  float v = 0;
  double u = 0;
  double u_prev = 0;
  float e_prev = 0;
  float e_prev2 = 0;
  int32_t pos_prev = 0;
  float targetRads = 0;  // Renamed from setRads
  float counts_per_rev = 4000.0;
  float a = 45;

  PID(setMotor* _motor, QEI* _enc, KalmanFilter* _kf, float _kp, float _ki, float _kd);  // Constructor that takes a setMotor object
  void compute(int32_t _p);
  void setRads(float _setRads);  // This remains the same
  void setK(float _kp, float _ki, float _kd);
};

#endif
