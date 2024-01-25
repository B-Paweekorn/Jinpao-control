#include "esp32-hal.h"
#ifndef GRIPPER_COMMAND_H
#define GRIPPER_COMMAND_H

#include "Arduino.h"

#include "Gripper_Config.h"
#include "Wire.h"
#include "MotionGenerator.h"

#define CURRENT_GAIN 9.0/145.0



class Gripper_command {

private:
  static const int NUM_MOTORS = 1;  // Number of motors, encoders, and PIDs
  static const int NUM_MOTORS_CUR = 2;  // Number of motors, encoders, and PIDs
  float dt = 1 / 1000.0;


  ESP32_CYTRON_MD* Mx[NUM_MOTORS + NUM_MOTORS_CUR];
  QEI* encx[NUM_MOTORS];
  PID_CONTROLLER* pidx_pos[NUM_MOTORS];
  PID_CONTROLLER* pidx_vel[NUM_MOTORS];
  PID_CONTROLLER* pidx_cur[NUM_MOTORS + NUM_MOTORS_CUR];
  DC_MOTOR_FFD* ffdx[NUM_MOTORS];
  KalmanFilter* kfx[NUM_MOTORS];
  MotionGenerator* tpx[NUM_MOTORS];
  CurrentFeedback* cfbx[NUM_MOTORS];


  uint32_t timestamp[NUM_MOTORS];
  float target[NUM_MOTORS];

  void ramp(float set_target, uint8_t index);

public:
  bool gripper1_status = false; 
  bool gripper2_status = false; 
  float q_target[NUM_MOTORS];
  float qd_target[NUM_MOTORS];
  float i_target[NUM_MOTORS];
  float i_gain[NUM_MOTORS] = { (9.0 / 145.0) };
  float ppr[NUM_MOTORS] = { 8192.0 };

  float cmd_vx[NUM_MOTORS];
  int16_t cmd_ux[NUM_MOTORS+NUM_MOTORS_CUR];
  double fb_q[NUM_MOTORS];
  float fb_qd[NUM_MOTORS];
  float fb_i[NUM_MOTORS+NUM_MOTORS_CUR];
  float fb_q_raw[NUM_MOTORS];
  float fb_q_pulse[NUM_MOTORS];

  // Gripper_command(ESP32_CYTRON_MD* _Mx[], PID_CONTROLLER* _pidx_pos[], PID_CONTROLLER* _pidx_vel[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], MotionGenerator* _tpx[]);
  Gripper_command(ESP32_CYTRON_MD *_Mx[], QEI *_encx[], PID_CONTROLLER *_pidx_pos[], PID_CONTROLLER *_pidx_vel[], PID_CONTROLLER *_pidx_cur[], DC_MOTOR_FFD *_ffdx[], KalmanFilter *_kfx[], MotionGenerator *_tpx[], CurrentFeedback* _cfbx[]);

  void begin();

  void setGoal(uint8_t M_index, float targetPosition);
  void setCur(uint8_t M_index, float targetCurrent);
  void setGrip(uint8_t M_index, int8_t grip_command);
  void tune(uint8_t M_index, float target, uint8_t tuining_state );
};

#endif
