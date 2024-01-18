#include "Cytron_Motor_430rpm_7W.h"

MotorConstant_Structure CYTRON_MOTOR_430RPM_7W_Constant = {
  .Ke =  0.215976429,
  .Kt = 0.215976429 / 2.0,
  .L = 1342.439024/1000000.0,
  .R = 2.7024,
  .J = 0.00023296087120213730022628015813059,
  .B = 0.00048787464715970046898829757256989,
  .V_max = 12.0,
  .U_max = 16383.0,
  .qdd_max = 60.0,
  .qd_max = 30.0 //rad/s
};

float CYTRON_MOTOR_430RPM_7W_MatrixA[16] = { 1.0, 9.909469514910439E-04, -0.002135455362998, 1.303638848870046E-04,
                     0.0, 0.976951313207159, -4.253705553115017, 0.196920745969728,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, -0.068345493119221, 0.194219450651197, 0.122669592832640 };

//input matrix
float CYTRON_MOTOR_430RPM_7W_MatrixB[4] = { 3.709300137196188E-05,
                    0.097109725325598,
                    0.0,
                    0.316887579732965 };

// observation matrix
float CYTRON_MOTOR_430RPM_7W_MatrixC[4] = { 1.0, 0.0, 0.0, 0.0 };

//process model variance vaule
float CYTRON_MOTOR_430RPM_7W_MatrixQ[1] = { 1 };

// measurement covariance matrix
float CYTRON_MOTOR_430RPM_7W_MatrixR[1] = { 0.01 };//0.0001
