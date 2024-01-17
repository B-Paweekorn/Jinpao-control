#include "Manipulator_Config.h"
#include "MotionGenerator.h"

// Adafruit_MCP23X17 mcp;

// Homing_controller hc1(mcp,MOTOR_1_HOME_PIN,1,TIMEOUT_MOTOR_X,SPEED_MOTOR_X);
// Homing_controller hc2(mcp,MOTOR_2_HOME_PIN,1,TIMEOUT_MOTOR_X,SPEED_MOTOR_X);
// Homing_controller hc3(mcp,MOTOR_3_HOME_PIN,1,TIMEOUT_MOTOR_X,SPEED_MOTOR_X);
// Homing_controller hcH(mcp,MOTOR_H_HOME_PIN,1,TIMEOUT_MOTOR_H,SPEED_MOTOR_H);

MotionGenerator tpH(5, CYTRON_MOTOR_680RPM_250W_Constant.qdd_max, 0);
MotionGenerator tp1(5, FAULHABER_2342L012CR_Constant.qdd_max, 0);
MotionGenerator tp2(5, FAULHABER_2342L012CR_Constant.qdd_max, 0);
MotionGenerator tp3(5, FAULHABER_2342L012CR_Constant.qdd_max, 0);

QEI encH(ENC_H_A_PIN, ENC_H_B_PIN, ENC_H_PPR);
QEI enc1(ENC_1_A_PIN, ENC_1_B_PIN, ENC_1_PPR);
QEI enc2(ENC_2_A_PIN, ENC_2_B_PIN, ENC_2_PPR);
QEI enc3(ENC_3_A_PIN, ENC_3_B_PIN, ENC_3_PPR);

ESP32_CYTRON_MD MH(MOTOR_H_PWM_PIN, MOTOR_H_DIR_PIN);
ESP32_CYTRON_MD M1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN);
ESP32_CYTRON_MD M2(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN);
ESP32_CYTRON_MD M3(MOTOR_3_PWM_PIN, MOTOR_3_DIR_PIN);

/*-----Config Motor End-----*/

/*-----Config Robot Base Start-----*/

/*-----Config Robot Base End-----*/

/*-----Config Kalman Start-----*/
KalmanFilter kfH(CYTRON_MOTOR_680RPM_250W_MatrixA,
                 CYTRON_MOTOR_680RPM_250W_MatrixB,
                 CYTRON_MOTOR_680RPM_250W_MatrixC,
                 CYTRON_MOTOR_680RPM_250W_MatrixQ,
                 CYTRON_MOTOR_680RPM_250W_MatrixR);
KalmanFilter kf1(FAULHABER_2342L012CR_MatrixA,
                 FAULHABER_2342L012CR_MatrixB,
                 FAULHABER_2342L012CR_MatrixC,
                 FAULHABER_2342L012CR_MatrixQ,
                 FAULHABER_2342L012CR_MatrixR);
KalmanFilter kf2(FAULHABER_2342L012CR_MatrixA,
                 FAULHABER_2342L012CR_MatrixB,
                 FAULHABER_2342L012CR_MatrixC,
                 FAULHABER_2342L012CR_MatrixQ,
                 FAULHABER_2342L012CR_MatrixR);
KalmanFilter kf3(FAULHABER_2342L012CR_MatrixA,
                 FAULHABER_2342L012CR_MatrixB,
                 FAULHABER_2342L012CR_MatrixC,
                 FAULHABER_2342L012CR_MatrixQ,
                 FAULHABER_2342L012CR_MatrixR);

/*-----Config Kalman End-----*/

/*-----Config Controller Start-----*/

PID_CONTROLLER pidH_pos(CYTRON_MOTOR_680RPM_250W_POSITION_KP,
                        CYTRON_MOTOR_680RPM_250W_POSITION_KI,
                        CYTRON_MOTOR_680RPM_250W_POSITION_KD);
PID_CONTROLLER pid1_pos(FAULHABER_2342L012CR_POSITION_KP,
                        FAULHABER_2342L012CR_POSITION_KI,
                        FAULHABER_2342L012CR_POSITION_KD);
PID_CONTROLLER pid2_pos(FAULHABER_2342L012CR_POSITION_KP,
                        FAULHABER_2342L012CR_POSITION_KI,
                        FAULHABER_2342L012CR_POSITION_KD);
PID_CONTROLLER pid3_pos(FAULHABER_2342L012CR_POSITION_KP,
                        FAULHABER_2342L012CR_POSITION_KI,
                        FAULHABER_2342L012CR_POSITION_KD);

PID_CONTROLLER pidH_vel(CYTRON_MOTOR_680RPM_250W_VElOCITY_KP,
                        CYTRON_MOTOR_680RPM_250W_VElOCITY_KI,
                        CYTRON_MOTOR_680RPM_250W_VElOCITY_KD);
PID_CONTROLLER pid1_vel(FAULHABER_2342L012CR_VElOCITY_KP,
                        FAULHABER_2342L012CR_VElOCITY_KI,
                        FAULHABER_2342L012CR_VElOCITY_KD);
PID_CONTROLLER pid2_vel(FAULHABER_2342L012CR_VElOCITY_KP,
                        FAULHABER_2342L012CR_VElOCITY_KI,
                        FAULHABER_2342L012CR_VElOCITY_KD);
PID_CONTROLLER pid3_vel(FAULHABER_2342L012CR_VElOCITY_KP,
                        FAULHABER_2342L012CR_VElOCITY_KI,
                        FAULHABER_2342L012CR_VElOCITY_KD);

DC_MOTOR_FFD ffdH(&CYTRON_MOTOR_680RPM_250W_Constant);
DC_MOTOR_FFD ffd1(&FAULHABER_2342L012CR_Constant);
DC_MOTOR_FFD ffd2(&FAULHABER_2342L012CR_Constant);
DC_MOTOR_FFD ffd3(&FAULHABER_2342L012CR_Constant);

/*-----Config Controller Start-----*/

ESP32_CYTRON_MD* Mx[4] = { &MH, &M1, &M2, &M3  };
QEI* encx[4] = { &encH, &enc1, &enc2, &enc3  };
PID_CONTROLLER* pidx_pos[4] = { &pidH_pos, &pid1_pos, &pid2_pos, &pid3_pos  };
PID_CONTROLLER* pidx_vel[4] = { &pidH_vel, &pid1_vel, &pid2_vel, &pid3_vel  };
DC_MOTOR_FFD* ffdx[4] = { &ffdH, &ffd1, &ffd2, &ffd3  };
KalmanFilter* kfx[4] = { &kfH, &kf1, &kf2, &kf3  };
MotionGenerator* tpx[4] = { &tpH, &tp1, &tp2, &tp3  };
// Homing_controller* hcx[4] = { &hc1, &hc2, &hc3, &hcH };

