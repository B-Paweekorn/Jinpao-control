#include "HardwareSerial.h"
#include <stdint.h>
#include "Gripper_Config.h"
#include "Wire.h"
#include "esp32-hal.h"
#include "Gripper_command.h"

Gripper_command::Gripper_command(ESP32_CYTRON_MD *_Mx[], QEI *_encx[], PID_CONTROLLER *_pidx_pos[], PID_CONTROLLER *_pidx_vel[], PID_CONTROLLER *_pidx_cur[], DC_MOTOR_FFD *_ffdx[], KalmanFilter *_kfx[], MotionGenerator *_tpx[], CurrentFeedback *_cfbx[]) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    encx[i] = _encx[i];
    pidx_pos[i] = _pidx_pos[i];
    pidx_vel[i] = _pidx_vel[i];
    ffdx[i] = _ffdx[i];
    kfx[i] = _kfx[i];
    tpx[i] = _tpx[i];
    cfbx[i] = _cfbx[i];
  }
  for (int i = 0; i < NUM_MOTORS + NUM_MOTORS_CUR; ++i) {
    Mx[i] = _Mx[i];
    pidx_cur[i] = _pidx_cur[i];
  }
}

void Gripper_command::begin() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    encx[i]->begin();
    cfbx[i]->begin();
  }

  // Wire.begin(ADC_SDA, ADC_SCL, 400000);

  // Wire1.begin(BNO_SDA, BNO_SCL, 400000);
  // while (!bno.begin()) vTaskDelay(10 / portTICK_PERIOD_MS);
  // bno.setExtCrystalUse(true);

  delay(10);

  for (int i = 0; i < NUM_MOTORS; i++) {
    fb_q[i] = 0;
    fb_qd[i] = 0;

    encx[i]->reset();
    kfx[i]->begin();
  }

  for (int i = 0; i < NUM_MOTORS + NUM_MOTORS_CUR; i++) {
    Mx[i]->begin();
    Mx[i]->set_duty(0);

    fb_i[i] = 0;

    cmd_ux[i] = 0;
  }

  delay(10);
}

void Gripper_command::setGoal(uint8_t M_index, float targetPosition) {

  static float dof0_prev_targetpos = -1;
  static bool break_flag = false;

  if (M_index == 0 && dof0_prev_targetpos != targetPosition && break_flag == true) {
    dof0_prev_targetpos = targetPosition;
    break_flag = false;
    //digitalWrite(10, LOW);
  }

  q_target[M_index] = tpx[M_index]->update(targetPosition);
  qd_target[M_index] = tpx[M_index]->getVelocity();

  fb_q[M_index] += encx[M_index]->get_diff_count() * 2 * M_PI / ppr[M_index];
  float *kf_ptr = kfx[M_index]->Compute(fb_q[M_index], cmd_ux[M_index] * ffdx[M_index]->Vmax / ffdx[M_index]->Umax);

  fb_qd[M_index] = kf_ptr[1];
  fb_i[M_index] = kf_ptr[3];

  if (abs(targetPosition - fb_q[M_index]) > 0.01) {

    cmd_vx[M_index] = pidx_pos[M_index]->Compute(q_target[M_index] - fb_q[M_index]);

    if (qd_target[M_index] + cmd_vx[M_index] != 0) {

      cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] + cmd_vx[M_index] - fb_qd[M_index]) + ffdx[M_index]->Compute(qd_target[M_index], i_gain[M_index] * fb_i[M_index]), ffdx[M_index]->Umax, -1 * ffdx[M_index]->Umax);

    } else {
      cmd_ux[M_index] = 0;
    }
  } else {
    if (M_index == 0) break_flag = true;
    cmd_ux[M_index] = 0;
  }

  if (M_index == 0 && break_flag == true) {
    //digitalWrite(10, HIGH);
    cmd_ux[M_index] = 0;
  }

  Mx[M_index]->set_duty(cmd_ux[M_index]);
}

void Gripper_command::setCur(uint8_t M_index, float targetCurrent) {

  // i_target[M_index] = tpx[M_index]->update(targetCurrent);

  float *current = cfbx[0]->read_current();

  fb_i[M_index] = (1000 * fb_i[M_index] / 1300.0) + (300.0 * current[M_index - 1] / 1300.0);

  if (targetCurrent != 0) {
    // cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] - fb_qd[M_index]), ffdx[M_index]->Umax, -1 * ffdx[M_index]->Umax);
    cmd_ux[M_index] = PWM_Satuation(pidx_cur[M_index]->Compute(targetCurrent - fb_i[M_index]), 16383, -16383);  //
  } else {
    cmd_ux[M_index] = 0;
  }
}

void Gripper_command::setGrip(uint8_t M_index, int8_t grip_command) {
  if (M_index == 1) {
    if (grip_command == 1) {
      setCur(1, 1);
      Serial.println("Gripper 1: grip");
    } else if (grip_command == -1) {
      if (fb_i[1] > 0.6 && M_index == 1) {
        gripper1_status = true;
      } else if (gripper1_status == false && M_index == 1) {
        Mx[1]->set_duty(-8000);
        Serial.println("Gripper 1: release");
      } else {
        grip_command = 0;
        gripper1_status = false;
        Mx[1]->set_duty(0);
        Serial.println("Gripper 1: off");
      }
    }
  } else if (M_index == 2) {
    if (grip_command == 1) {
      if (fb_i[2] > 0.8 && M_index == 2) {
        gripper2_status = true;
      } else if (gripper2_status == false && M_index == 2) {
        Mx[2]->set_duty(4000);
        Serial.println("Gripper 2: grip");
      } else {
        Mx[2]->set_duty(0);
        Serial.println("Gripper 2: off");
      }
    } else if (grip_command == 2) {
      setCur(2, 0.5);
      Serial.println("Gripper 2: hold");
    } else if (grip_command == 0) {
      Mx[2]->set_duty(0);
      Serial.println("Gripper 2: off");
    }
  }
}


void Gripper_command::tune(uint8_t M_index, float target, uint8_t tuining_state) {

  q_target[M_index] = tpx[M_index]->update(target);
  qd_target[M_index] = tpx[M_index]->getVelocity();

  fb_q[M_index] += encx[M_index]->get_diff_count() * 2 * M_PI / encx[M_index]->pulse_per_rev;
  float *kf_ptr = kfx[M_index]->Compute(fb_q[M_index], cmd_ux[M_index] * ffdx[M_index]->Vmax / ffdx[M_index]->Umax);

  fb_qd[M_index] = kf_ptr[1];
  fb_i[M_index] = kf_ptr[3];

  if (q_target[M_index] != 0) {
    cmd_vx[M_index] = pidx_pos[M_index]->Compute(q_target[M_index] - fb_q[M_index]);
  } else {
    cmd_vx[M_index] = 0;
  }

  // static int16_t u_duty;

  if (qd_target[M_index] != 0) {
    // cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] - fb_qd[M_index]), ffdx[M_index]->Umax, -1 * ffdx[M_index]->Umax);
    cmd_ux[M_index] = PWM_Satuation(pidx_vel[M_index]->Compute(qd_target[M_index] + cmd_vx[M_index] - fb_qd[M_index]) + ffdx[M_index]->Compute(qd_target[M_index], CURRENT_GAIN * fb_i[M_index]), ffdx[M_index]->Umax, -1 * ffdx[M_index]->Umax);  //
  } else {
    cmd_ux[M_index] = 0;
  }

  // if (tuining_state == 'V') u_duty = cmd_ux[M_index];
  // // else if (tuining_state == 'K')
  // u_duty = target;
  //cmd_ux[M_index] = target;
  Mx[M_index]->set_duty(cmd_ux[M_index]);  // cmd_ux[M_index]
}