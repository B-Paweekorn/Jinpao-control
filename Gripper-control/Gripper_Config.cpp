#include "Gripper_Config.h"
#include "MotionGenerator.h"

MotionGenerator tp1(CYTRON_MOTOR_430RPM_7W_Constant.qd_max, CYTRON_MOTOR_430RPM_7W_Constant.qdd_max, 0);
// MotionGenerator tp1(30, 1, 0);

QEI enc1(ENC_1_A_PIN, ENC_1_B_PIN, ENC_1_PPR);

ESP32_CYTRON_MD M1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN);
ESP32_CYTRON_MD M2(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN);
ESP32_CYTRON_MD M3(MOTOR_3_PWM_PIN, MOTOR_3_DIR_PIN);
/*-----Config Motor End-----*/

/*-----Config ADS1115 Start-----*/
CurrentFeedback cfb1(ADS_SDA_PIN, ADS_SCL_PIN);
/*-----Config ADS1115 End-----*/

/*-----Config Robot Base Start-----*/

/*-----Config Robot Base End-----*/

/*-----Config Kalman Start-----*/
KalmanFilter kf1(CYTRON_MOTOR_430RPM_7W_MatrixA,
                 CYTRON_MOTOR_430RPM_7W_MatrixB,
                 CYTRON_MOTOR_430RPM_7W_MatrixC,
                 CYTRON_MOTOR_430RPM_7W_MatrixQ,
                 CYTRON_MOTOR_430RPM_7W_MatrixR);
/*-----Config Kalman End-----*/

/*-----Config Controller Start-----*/

PID_CONTROLLER pid1_pos(CYTRON_MOTOR_430RPM_7W_POSITION_KP,
                        CYTRON_MOTOR_430RPM_7W_POSITION_KI,
                        CYTRON_MOTOR_430RPM_7W_POSITION_KD);

PID_CONTROLLER pid1_vel(CYTRON_MOTOR_430RPM_7W_VElOCITY_KP,
                        CYTRON_MOTOR_430RPM_7W_VElOCITY_KI,
                        CYTRON_MOTOR_430RPM_7W_VElOCITY_KD);

PID_CONTROLLER pid1_cur(GRIP_MOTOR_CURRENT_KP,
                        GRIP_MOTOR_CURRENT_KI,
                        GRIP_MOTOR_CURRENT_KD);

PID_CONTROLLER pid2_cur(FLAG_MOTOR_CURRENT_KP,
                        FLAG_MOTOR_CURRENT_KI,
                        FLAG_MOTOR_CURRENT_KD);

DC_MOTOR_FFD ffd1(&CYTRON_MOTOR_430RPM_7W_Constant);
/*-----Config Controller Start-----*/

CurrentFeedback* cfbx[1] = { &cfb1 };
ESP32_CYTRON_MD* Mx[3] = { &M1, &M2, &M3};
QEI* encx[1] = { &enc1 };
PID_CONTROLLER* pidx_pos[1] = { &pid1_pos };
PID_CONTROLLER* pidx_vel[1] = { &pid1_vel };
PID_CONTROLLER* pidx_cur[3] = { 0, &pid1_cur, &pid2_cur };
DC_MOTOR_FFD* ffdx[1] = { &ffd1 };
KalmanFilter* kfx[1] = { &kf1 };
MotionGenerator* tpx[1] = { &tp1 };
