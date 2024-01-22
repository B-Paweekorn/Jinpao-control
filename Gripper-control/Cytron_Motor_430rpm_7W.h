#ifndef __CYTRON_MOTOR_430RPM_7W_H__
#define __CYTRON_MOTOR_430RPM_7W_H__

#include "stdint.h"
#include "Controller.h"

#define CYTRON_MOTOR_430RPM_7W_POSITION_KP 17.3
#define CYTRON_MOTOR_430RPM_7W_POSITION_KI 0.004
#define CYTRON_MOTOR_430RPM_7W_POSITION_KD 1.8

#define CYTRON_MOTOR_430RPM_7W_VElOCITY_KP 300.0
#define CYTRON_MOTOR_430RPM_7W_VElOCITY_KI 5.0
#define CYTRON_MOTOR_430RPM_7W_VElOCITY_KD 0.0

#define AMT103_PPR 8192.0

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