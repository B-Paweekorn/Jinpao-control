#include "PID.h"

PID::PID(setMotor* _motor, QEI* _enc, float _kp, float _ki, float _kd) {
  motor = _motor;
  enc = _enc;
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

void PID::setK(float _kp, float _ki, float _kd) {
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

void PID::setRads(float _setRads) {
  if (_setRads > targetRads) {         
    targetRads = targetRads + a * dt;  
  } else if (_setRads < targetRads) {  
    targetRads = targetRads - a * dt;  
  }
}
void PID::compute() {

  int pos = enc->get_diff_count();

  float velocity = (pos - pos_prev) / dt;
  v = velocity / counts_per_rev * 2 * M_PI;

  float e = targetRads - v;

  u = u_prev + (kp + ki + kd) * e + (kp + 2 * kd) * e_prev + kd * e_prev2;

  if (u > 16383) {
    u = 16383;
  } else if (u < -16383) {
    u = -16383;
  }
  if (targetRads == 0) { u = 0; }
  //setSpeed(u);
  motor->setPWM(u);

  pos_prev = pos;
  e_prev = e;
  e_prev2 = e_prev;
  u_prev = u;
}
