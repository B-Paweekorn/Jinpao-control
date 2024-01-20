#include "Cytron_Motor_430rpm_7W.h"

MotorConstant_Structure CYTRON_MOTOR_430RPM_7W_Constant = {
  .Ke = 0.0134,
  .Kt = 0.0134,
  .L = 65.0e-6,
  .R = 0.6367,
  .J = 5.7e-7,
  .B = 0.00706757271,
  .V_max = 12.0,
  .U_max = 16383.0,
  .qdd_max = 22.0,
  .qd_max = 11.0 //rad/s
};

float CYTRON_MOTOR_430RPM_7W_MatrixA[16] = { 1.0, 7.958559454481690E-05, -0.128575229449185, 6.400636010723209E-05,
                     0.0, 3.130204756995285E-06, -1.396238500786261E+02, 4.449457574775939E-06,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, -3.901832027111207E-08, 0.984713232418955, -5.546267474529575E-08 };

//input matrix
float CYTRON_MOTOR_430RPM_7W_MatrixB[4] = { 8.731061655325501E-04,
                    0.984713232418956,
                    0.0,
                    0.519370999025400 };

// observation matrix
float CYTRON_MOTOR_430RPM_7W_MatrixC[4] = { 1.0, 0.0, 0.0, 0.0 };

//process model variance vaule
float CYTRON_MOTOR_430RPM_7W_MatrixQ[1] = { 1 };

// measurement covariance matrix
float CYTRON_MOTOR_430RPM_7W_MatrixR[1] = { 0.001 };//0.0001
