#ifndef __CYTRON_MOTOR_260RPM_250W_H__
#define __CYTRON_MOTOR_260RPM_250W_H__

#include <stdint.h>
#include <Controller.h>

#define CYTRON_MOTOR_260RPM_250W_VElOCITY_KP 7.5
#define CYTRON_MOTOR_260RPM_250W_VElOCITY_KI 7.5
#define CYTRON_MOTOR_260RPM_250W_VElOCITY_KD 0.0

#define AMT103_PPR 2048.0 * 4.0

MotorConstant_Structure CYTRON_MOTOR_260RPM_250W_Constant = {
  .Ke = 0.5265,
  .Kt = 0.5265 / 2.0,
  .L = 183.67e-6,
  .R = 0.6367,
  .J = 0.013369679361673,
  .B = 0.019688440522932,
  .V_max = 18.0
};

float CYTRON_MOTOR_260RPM_250W_MatrixA[16] = { 1.0, 9.945211490311812E-4, -3.727956981163958E-5, 4.075838177204351E-6,
                     0.0, 0.98685183868228, -0.074386312650263, 0.005453127455664,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, -0.79388648772862, 0.044382187370876, 0.027207395024798 };

//input matrix
float CYTRON_MOTOR_260RPM_250W_MatrixB[4] = { 9.012107076660543E-06,
                    0.022191093685438,
                    0.0,
                    1.509516246505134 };

// observation matrix
float CYTRON_MOTOR_260RPM_250W_MatrixC[4] = { 1.0, 0.0, 0.0, 0.0 };

//process model variance vaule
float CYTRON_MOTOR_260RPM_250W_MatrixQ[1] = { 1 };

// measurement covariance matrix
float CYTRON_MOTOR_260RPM_250W_MatrixR[1] = { 0.0001 };

#endif