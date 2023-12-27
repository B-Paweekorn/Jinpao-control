#ifndef _MOBILE_COMMAND_H
#define _MOBILE_COMMAND_H

class Mobile_command
{
    private:
      float32_t v_x;
      float32_t v_y;
      float32_t omega_z;
    public:
      SetRobotvel(float32_t v_x, float32_t v_y, float32_t omega_z);     
};

#endif