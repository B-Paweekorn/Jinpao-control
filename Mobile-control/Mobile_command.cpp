#include "Mobile_command.h"

Mobile_command::Mobile_command(ESP32_CYTRON_MD* _Mx[], QEI* _encx[], PID_CONTROLLER* _pidx[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], Kinematics* _kinematics)
  : kinematics(_kinematics) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    Mx[i] = _Mx[i];
    encx[i] = _encx[i];
    pidx[i] = _pidx[i];
    ffdx[i] = _ffdx[i];
    kfx[i] = _kfx[i];
  }
}

void Mobile_command::begin() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    encx[i]->begin();
  }

  delay(10);

  for (int i = 0; i < NUM_MOTORS; i++) {
    Mx[i]->begin();
    Mx[i]->set_duty(0);

    kfx[i]->begin();

    cmd_ux[i] = 0;
    fb_q[i] = 0;
    fb_qd[i] = 0;
    fb_i[i] = 0;
  }
}

void Mobile_command::control(float _vx, float _vy, float _wz) {
  Kinematics::RadPS wheel_radps = kinematics->Inverse_Kinematics(_vx, _vy, _wz);
  float target[NUM_MOTORS] = {
    wheel_radps.radps_fl,
    wheel_radps.radps_fr,
    wheel_radps.radps_bl,
    wheel_radps.radps_br
  };

  for (int i = 0; i < NUM_MOTORS; i++) {
    fb_q[i] += encx[i]->get_diff_count() * 2 * M_PI / encx[i]->pulse_per_rev;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    float* kf_ptr = kfx[i]->Compute(fb_q[i],
                                    cmd_ux[i] * ffdx[i]->Vmax / PWM_STATUATION_VALUE);
    fb_qd[i] = kf_ptr[1];
    fb_i[i] = kf_ptr[3];

    cmd_ux[i] = PWM_Satuation(pidx[i]->Compute(target[i] - fb_qd[i]) + ffdx[i]->Compute(target[i], fb_i[i]),
                           PWM_STATUATION_VALUE,
                           -1 * PWM_STATUATION_VALUE);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    Mx[i]->set_duty(cmd_ux[i]);
  }
}
