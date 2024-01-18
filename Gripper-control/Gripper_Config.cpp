#include "Gripper_Config.h"
#include "MotionGenerator.h"

MotionGenerator tp1(CYTRON_MOTOR_430RPM_7W_Constant.qd_max, CYTRON_MOTOR_430RPM_7W_Constant.qdd_max, 0);
// MotionGenerator tp1(30, 1, 0);

QEI enc1(ENC_1_A_PIN, ENC_1_B_PIN, ENC_1_PPR);

ESP32_CYTRON_MD M1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN);
/*-----Config Motor End-----*/

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

DC_MOTOR_FFD ffd1(&CYTRON_MOTOR_430RPM_7W_Constant);
/*-----Config Controller Start-----*/

ESP32_CYTRON_MD* Mx[1] = { &M1 };
QEI* encx[1] = { &enc1 };
PID_CONTROLLER* pidx_pos[1] = { &pid1_pos };
PID_CONTROLLER* pidx_vel[1] = { &pid1_vel };
DC_MOTOR_FFD* ffdx[1] = { &ffd1 };
KalmanFilter* kfx[1] = { &kf1 };
MotionGenerator* tpx[1] = { &tp1 };
