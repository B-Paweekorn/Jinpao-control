#include "Cytron_Motor_430rpm_7W.h"

MotorConstant_Structure CYTRON_MOTOR_430RPM_7W_Constant = {
  .Ke = 0.201460170000000,
  .Kt = 0.100730085000000,
  .L = 1.624060150375940E-04,
  .R = 0.604511278195489,
  .J = 0.002131517773002,
  .B = 0.001114942610956,
  .V_max = 18.0,
  .U_max = 16383.0,
  .qdd_max = 90.0, //90.0,
  .qd_max = 42.0//42.0 //rad/s
};

float CYTRON_MOTOR_430RPM_7W_MatrixA[16] = { 1.0, 9.947641097582700E-04, -2.338658274849931E-04, 9.330414794773396E-06,
                     0.0, 0.987438849897746, -0.466692852557011, 0.012280133415304,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, -0.322344250901005, 0.114902330343043, 0.020454698259843 };


//input matrix
float CYTRON_MOTOR_430RPM_7W_MatrixB[4] = { 2.353456486180652E-05,
                    0.057451165171521,
                    0.0,
                    1.601245857856400 };
// observation matrix
float CYTRON_MOTOR_430RPM_7W_MatrixC[4] = { 1.0, 0.0, 0.0, 0.0 };

//process model variance vaule
float CYTRON_MOTOR_430RPM_7W_MatrixQ[1] = { 1 };

// measurement covariance matrix
float CYTRON_MOTOR_430RPM_7W_MatrixR[1] = { 0.001 };//0.0001
