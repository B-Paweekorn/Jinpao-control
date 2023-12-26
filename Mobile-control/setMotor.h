#include <inttypes.h>

class setMotor
{
    private:
      int16_t PWM;  //PWM vaule
      uint8_t PWM_PIN;  
      uint8_t DIR_PIN;
      uint32_t MOTOR_BASE_FREQ; //PWM frequency
      uint16_t MOTOR_TIMER_14_BIT;  //PWM resolution
    public:
      setMotor(int16_t _PWM_PIN, uint8_t _DIR_PIN, uint32_t _MOTOR_BASE_FREQ, uint16_t _MOTOR_TIMER_14_BIT);  //constructor
      void begin(); //set pin mode
      void setPWM(int16_t PWM); //set motor's PWM 
};