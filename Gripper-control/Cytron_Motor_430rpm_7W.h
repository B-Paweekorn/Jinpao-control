#ifndef __CYTRON_MOTOR_430RPM_7W_H__
#define __CYTRON_MOTOR_430RPM_7W_H__

#include "stdint.h"
#include "Controller.h"

#define CYTRON_MOTOR_430RPM_7W_POSITION_KP 50
#define CYTRON_MOTOR_430RPM_7W_POSITION_KI 0.000005
#define CYTRON_MOTOR_430RPM_7W_POSITION_KD 0.0

#define CYTRON_MOTOR_430RPM_7W_VElOCITY_KP 4.0
#define CYTRON_MOTOR_430RPM_7W_VElOCITY_KI 10.0
#define CYTRON_MOTOR_430RPM_7W_VElOCITY_KD 0.0

#define FLAG_MOTOR_CURRENT_KP 15000.0 //10000.0
#define FLAG_MOTOR_CURRENT_KI 150.0 //100.0
#define FLAG_MOTOR_CURRENT_KD 0.0

#define GRIP_MOTOR_CURRENT_KP 10000.0 //10000.0
#define GRIP_MOTOR_CURRENT_KI 100.0 //100.0
#define GRIP_MOTOR_CURRENT_KD 0.0

#define AMT103_PPR 2048.0 * 4.0

extern MotorConstant_Structure CYTRON_MOTOR_430RPM_7W_Constant;

extern float CYTRON_MOTOR_430RPM_7W_MatrixA[];

//input matrix
extern float CYTRON_MOTOR_430RPM_7W_MatrixB[];

// observation matrix
extern float CYTRON_MOTOR_430RPM_7W_MatrixC[];

//process model variance vaule
extern float CYTRON_MOTOR_430RPM_7W_MatrixQ[];

// measurement covariance matrix
extern float CYTRON_MOTOR_430RPM_7W_MatrixR[];

#endif