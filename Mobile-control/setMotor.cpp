#include "esp32-hal-gpio.h"
#include "setMotor.h"
#include "math.h"
#include "Arduino.h"
setMotor::setMotor(int32_t PWM_PIN, uint8_t DIR_PIN, uint32_t MOTOR_BASE_FREQ, uint16_t MOTOR_TIMER_14_BIT)
{
    PWM_PIN = PWM_PIN;
    DIR_PIN = DIR_PIN;
    MOTOR_BASE_FREQ = MOTOR_BASE_FREQ;
    MOTOR_TIMER_14_BIT = MOTOR_TIMER_14_BIT;
    pinMode(DIR_PIN, OUTPUT);
    ledcAttach(PWM_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
}

void setMotor::setPWM(int32_t PWM)
{   
    if(PWM >= 0)
    {
      digitalWrite(DIR_PIN, 1);
    }
    else
    {
      digitalWrite(DIR_PIN, 0);
      PWM = abs(PWM);
    }
    ledcWrite(PWM_PIN, PWM);
}

