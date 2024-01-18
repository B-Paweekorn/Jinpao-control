#include "esp32-hal.h"
#ifndef GRIPPER_COMMAND_H
#define GRIPPER_COMMAND_H

#include "Arduino.h"

#include "Gripper_Config.h"
#include "Wire.h"
#include "MotionGenerator.h"

#define CURRENT_GAIN 1.0/12.5



class Gripper_command {

private:
  static const int NUM_MOTORS = 1;  // Number of motors, encoders, and PIDs
  float dt = 1 / 1000.0;


  ESP32_CYTRON_MD* Mx[NUM_MOTORS];
  QEI* encx[NUM_MOTORS];
  PID_CONTROLLER* pidx_pos[NUM_MOTORS];
  PID_CONTROLLER* pidx_vel[NUM_MOTORS];
  DC_MOTOR_FFD* ffdx[NUM_MOTORS];
  KalmanFilter* kfx[NUM_MOTORS];
  MotionGenerator* tpx[NUM_MOTORS];


  uint32_t timestamp[NUM_MOTORS];
  float target[NUM_MOTORS];

  void ramp(float set_target, uint8_t index);

public:
  float q_target[NUM_MOTORS];
  float qd_target[NUM_MOTORS];

  int16_t cmd_vx[NUM_MOTORS];
  int16_t cmd_ux[NUM_MOTORS];
  double fb_q[NUM_MOTORS];
  float fb_qd[NUM_MOTORS];
  float fb_i[NUM_MOTORS];

  // Gripper_command(ESP32_CYTRON_MD* _Mx[], PID_CONTROLLER* _pidx_pos[], PID_CONTROLLER* _pidx_vel[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], MotionGenerator* _tpx[]);
  Gripper_command(ESP32_CYTRON_MD* _Mx[], QEI* _encx[], PID_CONTROLLER* _pidx_pos[], PID_CONTROLLER* _pidx_vel[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], MotionGenerator* _tpx[]);

  void begin();

  void setGoal(uint8_t M_index, float targetPosition);
  void tune(uint8_t M_index, float target, uint8_t tuining_state );
};

#endif
