#include <stdint.h>
#include "PID.h"

PID::PID(setMotor* _motor, QEI* _enc, KalmanFilter* _kf, float _kp, float _ki, float _kd) {
  motor = _motor;
  enc = _enc;
  kp = _kp;
  ki = _ki;
  kd = _kd;
  kf = _kf;
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

float PID::scaleToPWM(float v_in) {
  // Constants for PWM scaling
  const float PWM_min = -16383;
  const float PWM_max = 16383;
  const float v_in_min = -18;
  const float v_in_max = 18;

  // Scale v_in to the PWM range
  float PWM = PWM_min + (v_in - v_in_min) / (v_in_max - v_in_min) * (PWM_max - PWM_min);

  // Limit PWM to its defined range
  PWM = std::max(PWM_min, std::min(PWM, PWM_max));


  return PWM;
}

float PID::calculateV_in(float omega, float i) {
  // Constants for the motor and system parameters
  const float R = 0.6367f;
  const float ke = 0.5265f;
  const float kt = ke / 2.0f;
  const float b = 0.019688440522932f;

  // Calculate v_in based on the formula
  float v_in = ke * omega + i * R;

  // Convert v_in to PWM
  return scaleToPWM(v_in);
}

void PID::compute() {


  int pos = enc->get_diff_count();

  float velocity = (pos - pos_prev) / dt;
  v = velocity / counts_per_rev * 2 * M_PI;

  v = kf->EstimateSpeed(v, u);

  // v = kf->EstimateSpeed(pos , u);

  float omega = targetRads;  // Replace with actual value
  float i = 0.0;             // Replace with actual value
  e = targetRads - v;
  // Calculate PWM
  PWM_feedforward = calculateV_in(omega, i);

  u += (kp + ki + kd) * e + (kp + 2 * kd) * e_prev + kd * e_prev2;
  u_out = u + PWM_feedforward;




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
