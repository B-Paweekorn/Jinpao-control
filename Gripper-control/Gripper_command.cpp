#include <stdint.h>
#include "Gripper_Config.h"
#include "Wire.h"
#include "esp32-hal.h"
#include "Gripper_command.h"

// Gripper_command::Gripper_command(ESP32_CYTRON_MD* _Mx[], PID_CONTROLLER* _pidx_pos[], PID_CONTROLLER* _pidx_vel[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], MotionGenerator* _tpx[]) {
//   for (int i = 0; i < NUM_MOTORS; ++i) {
//     Mx[i] = _Mx[i];
//     pidx_pos[i] = _pidx_pos[i];
//     pidx_vel[i] = _pidx_vel[i];
//     ffdx[i] = _ffdx[i];
//     kfx[i] = _kfx[i];
//     tpx[i] = _tpx[i];
//   }
// }

Gripper_command::Gripper_command(ESP32_CYTRON_MD *_Mx[], QEI *_encx[], PID_CONTROLLER *_pidx_pos[], PID_CONTROLLER *_pidx_vel[], DC_MOTOR_FFD *_ffdx[], KalmanFilter *_kfx[], MotionGenerator *_tpx[]) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    Mx[i] = _Mx[i];
    encx[i] = _encx[i];
    pidx_pos[i] = _pidx_pos[i];
    pidx_vel[i] = _pidx_vel[i];
    ffdx[i] = _ffdx[i];
    kfx[i] = _kfx[i];
    tpx[i] = _tpx[i];
  }
}

void Gripper_command::begin() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    encx[i]->begin();
  }

  // Wire.begin(ADC_SDA, ADC_SCL, 400000);

  // Wire1.begin(BNO_SDA, BNO_SCL, 400000);
  // while (!bno.begin()) vTaskDelay(10 / portTICK_PERIOD_MS);
  // bno.setExtCrystalUse(true);

  delay(10);

  for (int i = 0; i < NUM_MOTORS; i++) {
    Mx[i]->begin();
    Mx[i]->set_duty(0);

    cmd_ux[i] = 0;
    fb_q[i] = 0;
    fb_qd[i] = 0;
    fb_i[i] = 0;

    encx[i]->reset();
    kfx[i]->begin();
  }

  delay(10);
}

void Gripper_command::setGoal(uint8_t M_index, float targetPosition) {

  q_target[M_index] = tpx[M_index]->update(targetPosition);
  qd_target[M_index] = tpx[M_index]->getVelocity();

  fb_q[M_index] += encx[M_index]->get_diff_count() * 2 * M_PI / encx[M_index]->pulse_per_rev;
  float *kf_ptr = kfx[M_index]->Compute(fb_q[M_index], cmd_ux[M_index] * ffdx[M_index]->Vmax / ffdx[M_index]->Umax);

  fb_qd[M_index] = kf_ptr[1];
  fb_i[M_index] = kf_ptr[3];


  if (q_target[M_index] != 0)
  {  
    //cmd_vx[M_index] = PWM_Satuation(pidx_pos[M_index]->Compute(q_target[M_index] - fb_q[M_index]),
                                  // ffdx[M_index]->qdmax,
                                  // -1 * ffdx[M_index]->qdmax);
    cmd_vx[M_index] = pidx_pos[M_index]->Compute(q_target[M_index] - fb_q[M_index]);
  }
  else
  {
    cmd_vx[M_index] = 0;
  }

  // static int16_t u_duty;

  if (qd_target[M_index] + cmd_vx[M_index] != 0) {
    // cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] - fb_qd[M_index])+ffdx[i]->Compute(qd_target[i], CURRENT_GAIN * fb_i[i]), ffdx[M_index]->Umax, -1 * ffdx[M_index]->Umax);

    cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] + cmd_vx[M_index] - fb_qd[M_index])+ffdx[M_index]->Compute(qd_target[M_index], CURRENT_GAIN * fb_i[M_index]),
                                    ffdx[M_index]->Umax,
                                    -1 * ffdx[M_index]->Umax); //
  } else {
    cmd_ux[M_index] = 0;
  }

  // if (tuining_state == 'V') u_duty = cmd_ux[M_index];
  // // else if (tuining_state == 'K')
  // u_duty = target;

  Mx[M_index]->set_duty(cmd_ux[M_index]);  // cmd_ux[M_index]
}

void Gripper_command::tune(uint8_t M_index, float target, uint8_t tuining_state) {

  q_target[M_index] = tpx[M_index]->update(target);
  qd_target[M_index] = tpx[M_index]->getVelocity();

  fb_q[M_index] += encx[M_index]->get_diff_count() * 2 * M_PI / encx[M_index]->pulse_per_rev;
  float *kf_ptr = kfx[M_index]->Compute(fb_q[M_index], cmd_ux[M_index] * ffdx[M_index]->Vmax / ffdx[M_index]->Umax);

  fb_qd[M_index] = kf_ptr[1];
  fb_i[M_index] = kf_ptr[3];

  if (q_target[M_index] != 0)
  {
    cmd_vx[M_index] = pidx_pos[M_index]->Compute(q_target[M_index] - fb_q[M_index]);
  }
  else
  {
    cmd_vx[M_index] = 0;
  }

  // static int16_t u_duty;

  if (qd_target[M_index] != 0)
  {
    // cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] - fb_qd[M_index]), ffdx[M_index]->Umax, -1 * ffdx[M_index]->Umax);
    cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] + cmd_vx[M_index] - fb_qd[M_index])+ffdx[M_index]->Compute(qd_target[M_index], CURRENT_GAIN * fb_i[M_index]), ffdx[M_index]->Umax, -1 * ffdx[M_index]->Umax); //
  }
  else
  {
    cmd_ux[M_index] = 0;
  }

  // if (tuining_state == 'V') u_duty = cmd_ux[M_index];
  // // else if (tuining_state == 'K')
  // u_duty = target;
  //cmd_ux[M_index] = target;
  Mx[M_index]->set_duty(cmd_ux[M_index]);  // cmd_ux[M_index]
}