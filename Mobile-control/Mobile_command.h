#ifndef MOBILE_COMMAND_H
#define MOBILE_COMMAND_H

#include "QEI.h"
#include "setMotor.h"
#include "Kinematics.h"
#include "PID.h"
#include <inttypes.h>

class Mobile_command
{
    private:
      float v_x;
      float v_y;
      float omega;
    
    public:
      Mobile_command();
      

};
#endif