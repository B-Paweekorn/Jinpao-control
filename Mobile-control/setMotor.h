#include <inttypes.h>

class setMotor
{
    private:
      int32_t PWM;  //PWM vaule
      uint8_t PWM_PIN;  
      uint8_t DIR_PIN;
      uint32_t MOTOR_BASE_FREQ; //PWM frequency
      uint16_t MOTOR_TIMER_14_BIT;  //PWM resolution
    public:
      setMotor(int32_t PWM_PIN, uint8_t DIR_PIN, uint32_t MOTOR_BASE_FREQ, uint16_t MOTOR_TIMER_14_BIT);  //constructor
      void setPWM(int32_t PWM); //set motor's PWM 
};