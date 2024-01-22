#ifndef __MANIPULATOR_CONFIG_H__
#define __MANIPULATOR_CONFIG_H__

#include "QEI.h"
#include "KalmanFilter.h"
#include "Controller.h"
#include "Esp32_Cytron_MDxx.h"

#include "Cytron_Motor_680rpm_250W.h"
#include "Faulhaber_2342l012cr.h"
#include "MotionGenerator.h"
// #include <Adafruit_MCP23X17.h>
#include "Homing_controller.h"


/*-----Config MCP Start-----*/
#define MCP_SDA 15
#define MCP_SCL 16

#define MOTOR_1_HOME_PIN 8
#define MOTOR_2_HOME_PIN 9
#define MOTOR_3_HOME_PIN 10
#define MOTOR_H_HOME_PIN 12

#define MOTOR_H_BRAKE_PIN 10  //GPIO

/*-----Config MCP End-----*/

/*-----Config Homing Start-----*/
#define TIMEOUT_MOTOR_X 5000  //MS
#define SPEED_MOTOR_X -5000    //PWM
#define TIMEOUT_MOTOR_H 5000  //MS
#define SPEED_MOTOR_H -5000    //PWM
#define MOTOR_X_CURRENT_LIMIT 0
#define MOTOR_H_CURRENT_LIMIT 0
/*-----Config Homing End-----*/

/*-----Config ADC Start-----*/
#define ADC_SDA 13
#define ADC_SCL 14
/*-----Config ADC End-----*/

/*-----Config BNO Start-----*/
#define BNO_SDA 15
#define BNO_SCL 16
/*-----Config BNO End-----*/

/*-----Config Motor Start-----*/
#define MOTOR_H_PWM_PIN 39
#define MOTOR_1_PWM_PIN 41
#define MOTOR_2_PWM_PIN 2
#define MOTOR_3_PWM_PIN 5

#define MOTOR_H_DIR_PIN 40
#define MOTOR_1_DIR_PIN 42
#define MOTOR_2_DIR_PIN 1
#define MOTOR_3_DIR_PIN 4

#define ENC_H_A_PIN 18
#define ENC_H_B_PIN 17
#define ENC_1_A_PIN 33
#define ENC_1_B_PIN 34
#define ENC_2_A_PIN 36
#define ENC_2_B_PIN 35
#define ENC_3_A_PIN 37
#define ENC_3_B_PIN 38

#define ENC_H_PPR 2048.0 * 4.0
#define ENC_1_PPR 64.0 * 12.0 * 4.0
#define ENC_2_PPR 64.0 * 12.0 * 4.0
#define ENC_3_PPR 64.0 * 12.0 * 4.0

/*-----Config Motor End-----*/

/*-----Config Robot Base Start-----*/

/*-----Config Robot Base End-----*/

/*-----Config Kalman Start-----*/

/*-----Config Kalman End-----*/

/*-----Config Controller Start-----*/

/*-----Config Controller Start-----*/

extern ESP32_CYTRON_MD* Mx[];
extern QEI* encx[];
extern PID_CONTROLLER* pidx_pos[];
extern PID_CONTROLLER* pidx_vel[];
extern DC_MOTOR_FFD* ffdx[];
extern KalmanFilter* kfx[];
extern MotionGenerator* tpx[];
extern Homing_controller* hcx[];
extern Adafruit_MCP23X17 mcp;

#endif