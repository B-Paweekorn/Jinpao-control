#include QEI.h
#include <inttypes.h>

class Mobile_command
{
    private:
      float32_t v_x;
      float32_t v_y;
      float32_t omega;
      // output
      float32_t output[4];
    
    public:
      Mobile_command();

};