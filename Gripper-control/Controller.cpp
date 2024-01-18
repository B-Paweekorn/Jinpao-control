#include "Controller.h"

int32_t PWM_Satuation(float _u, int32_t _upper_limit, int32_t _lower_limit) {
  if (_u > _upper_limit) return _upper_limit;
  else if (_u < _lower_limit) return _lower_limit;
  return (int32_t) _u;
}

PID_CONTROLLER ::PID_CONTROLLER(float _Kp, float _Ki, float _Kd)
  : Kp(_Kp), Ki(_Ki), Kd(_Kd) {}

float PID_CONTROLLER ::Compute(float ek) {
  u += ((Kp + Ki + Kd) * ek) - ((Kp + (2 * Kd)) * ek_1) + (Kd * ek_2);
  ek_2 = ek_1;
  ek_1 = ek;
  return u;
}

DC_MOTOR_FFD ::DC_MOTOR_FFD(MotorConstant_Structure *_Mx)
  : Mx(_Mx) {
    Vmax = _Mx->V_max;
    Umax = _Mx->U_max;
    qdmax = _Mx->qd_max;
    qddmax = _Mx->qdd_max;
  }

float DC_MOTOR_FFD ::Compute(float qd, float i) {
  // float V = (Mx->Ke * qd);
  float V = (Mx->Ke * qd) + (Mx->R * i);
  return Umax * V / Vmax;
}