#include <exception>
#ifndef __GRIPPER_CONFIG_H__
#define __GRIPPER_CONFIG_H__

#include "QEI.h"
#include "KalmanFilter.h"
#include "Controller.h"
#include "Esp32_Cytron_MDxx.h"
#include "CurrentFeedback.h"

#include "Cytron_Motor_430rpm_7W.h"
#include "MotionGenerator.h"

// /*-----Config ADC Start-----*/
// #define ADC_SDA 13
// #define ADC_SCL 14
// /*-----Config ADC End-----*/

/*-----Config ADS1115 Start-----*/
#define ADS_SDA_PIN 13
#define ADS_SCL_PIN 14
/*-----Config ADS1115 End-----*/

// /*-----Config BNO Start-----*/
// #define BNO_SDA 15
// #define BNO_SCL 16
// /*-----Config BNO End-----*/

/*-----Config Motor Start-----*/
#define MOTOR_1_PWM_PIN 40
#define MOTOR_2_PWM_PIN 41
#define MOTOR_3_PWM_PIN 2

#define MOTOR_1_DIR_PIN 39
#define MOTOR_2_DIR_PIN 42
#define MOTOR_3_DIR_PIN 1


#define ENC_1_A_PIN 18
#define ENC_1_B_PIN 17

#define ENC_1_PPR 2048.0 * 4.0
/*-----Config Motor End-----*/

/*-----Config Robot Base Start-----*/

/*-----Config Robot Base End-----*/

/*-----Config Kalman Start-----*/

/*-----Config Kalman End-----*/

/*-----Config Controller Start-----*/

/*-----Config Controller Start-----*/

extern CurrentFeedback* cfbx[];
extern ESP32_CYTRON_MD* Mx[];
extern QEI* encx[];
extern PID_CONTROLLER* pidx_pos[];
extern PID_CONTROLLER* pidx_vel[];
extern PID_CONTROLLER* pidx_cur[];
extern DC_MOTOR_FFD* ffdx[];
extern KalmanFilter* kfx[];
extern MotionGenerator* tpx[];


#endif