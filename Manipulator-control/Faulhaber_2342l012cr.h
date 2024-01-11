#ifndef __FAULHABER_2342L012CR_H__
#define __FAULHABER_2342L012CR_H__

#include "stdint.h"
#include "Controller.h"

#define FAULHABER_2342L012CR_POSITION_KP 0.0 //7.5
#define FAULHABER_2342L012CR_POSITION_KI 0.0 //0.0
#define FAULHABER_2342L012CR_POSITION_KD 0.0

#define FAULHABER_2342L012CR_VElOCITY_KP 3000 //7.5
#define FAULHABER_2342L012CR_VElOCITY_KI 0 //7.5
#define FAULHABER_2342L012CR_VElOCITY_KD 0.0


#define AMT103_PPR 3072.0

extern MotorConstant_Structure FAULHABER_2342L012CR_Constant;

extern float FAULHABER_2342L012CR_MatrixA[];

//input matrix
extern float FAULHABER_2342L012CR_MatrixB[];

// observation matrix
extern float FAULHABER_2342L012CR_MatrixC[];

//process model variance vaule
extern float FAULHABER_2342L012CR_MatrixQ[];

// measurement covariance matrix
extern float FAULHABER_2342L012CR_MatrixR[];

#endif