#include "esp32-hal.h"
#ifndef MAINPULATOR_COMMAND_H
#define MAINPULATOR_COMMAND_H

#include "Arduino.h"

#include "Manipulator_Config.h"
#include "Wire.h"
#include "MotionGenerator.h"
#include "Homing_controller.h"

class Manipulator_command {

  Adafruit_MCP23X17& mcp;
private:
  static const int NUM_MOTORS = 4;  // Number of motors, encoders, and PIDs
  float dt = 1 / 1000.0;



  ESP32_CYTRON_MD* Mx[NUM_MOTORS];
  QEI* encx[NUM_MOTORS];
  PID_CONTROLLER* pidx_pos[NUM_MOTORS];
  PID_CONTROLLER* pidx_vel[NUM_MOTORS];
  DC_MOTOR_FFD* ffdx[NUM_MOTORS];
  KalmanFilter* kfx[NUM_MOTORS];
  MotionGenerator* tpx[NUM_MOTORS];
  Homing_controller* hcx[NUM_MOTORS];


  uint32_t timestamp[NUM_MOTORS];
  float target[NUM_MOTORS];


public:
  float q_target[NUM_MOTORS];
  float qd_target[NUM_MOTORS];
  float i_gain[NUM_MOTORS] = { (9.0 / 145.0), (9.0 / 145.0), (9.0 / 145.0), (9.0 / 145.0) };
  float ppr[NUM_MOTORS] = { 8192.0, 3072.0, 3072.0, 3072.0 };
  int16_t cmd_vx[NUM_MOTORS];
  int16_t cmd_ux[NUM_MOTORS];
  double fb_q[NUM_MOTORS];
  float fb_qd[NUM_MOTORS];
  float fb_i[NUM_MOTORS];
  float prev_targetPosition = 0;
  uint8_t isBreak = 0;

  Manipulator_command(ESP32_CYTRON_MD* _Mx[], QEI* _encx[], PID_CONTROLLER* _pidx_pos[], PID_CONTROLLER* _pidx_vel[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], MotionGenerator* _tpx[], Homing_controller* _hcx[], Adafruit_MCP23X17& mcpRef);

  void begin();

  void setGoal(uint8_t M_index, float targetPosition);
  void tunesetGoal(uint8_t M_index, float targetPosition);
  void setHomeAll();
  void pollHoming();
  void setHome(uint8_t M_index);
};

#endif
