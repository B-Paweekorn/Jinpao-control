#include "QEI.h"
#include "setMotor.h"
#include "Kinematics.h"
#include "PID.h"
/*-----QEI(encA, encB)------*/

QEI enc0(17, 18); 
QEI enc1(33, 34);
QEI enc2(35, 36);
QEI enc3(37, 38);

long counter0 = 0;
long counter1 = 0;
long counter2 = 0;
long counter3 = 0;

/*-----Config Motor -----*/

#define MOTOR_TIMER_14_BIT 14
#define MOTOR_BASE_FREQ 1000

#define MOTOR_1_PWM_PIN  39
#define MOTOR_2_PWM_PIN  42
#define MOTOR_3_PWM_PIN  1
#define MOTOR_4_PWM_PIN  4

#define MOTOR_1_PWM_CHANNEL 0
#define MOTOR_2_PWM_CHANNEL 1
#define MOTOR_3_PWM_CHANNEL 2
#define MOTOR_4_PWM_CHANNEL 3

#define MOTOR_1_DIR_PIN  40
#define MOTOR_2_DIR_PIN  41
#define MOTOR_3_DIR_PIN  2
#define MOTOR_4_DIR_PIN  5

setMotor MOTOR_1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);
setMotor MOTOR_2(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);
setMotor MOTOR_3(MOTOR_3_PWM_PIN, MOTOR_3_DIR_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);
setMotor MOTOR_4(MOTOR_4_PWM_PIN, MOTOR_4_DIR_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);

kalman M1()
 = M1.EstimatedSpeed();
/*-----Config ADC-----*/
#define ADC_SDA 13 
#define ADC_SCL 14

/*-----Config BNO-----*/
#define BNO_SDA 15
#define BNO_SCL 16

/*-----Config Robot Base-----*/
#define WHEEL_DIAMETER 25 //meter

Kinematics Robot();

void setup() {
  Serial.begin(115200);

  /*-----Setup Hardware Start-----*/
  
  MOTOR_1.begin();
  MOTOR_2.begin();
  MOTOR_3.begin();
  MOTOR_4.begin();

  enc0.begin();
  enc1.begin();
  enc2.begin();
  enc3.begin();

  /*-----Setup Hardware End-------*/
}

void loop() {

  /*-----Test PWM Start (0 - 16383)-----*/
  // %duty 0 - 16382
  MOTOR_1.setPWM(4096);
  MOTOR_2.setPWM(8192);
  MOTOR_3.setPWM(12288);
  MOTOR_4.setPWM(16383);

  /*-----Test PWM End-------*/

  uint32_t time = micros();

  counter0 += enc0.get_diff_count();
  counter1 += enc1.get_diff_count();
  counter2 += enc2.get_diff_count();
  counter3 += enc3.get_diff_count();

  uint32_t dt = micros() - time;

  Serial.print(dt);
  Serial.print(' ');
  Serial.print(counter0);
  Serial.print(' ');
  Serial.print(counter1);
  Serial.print(' ');
  Serial.print(counter2);
  Serial.print(' ');
  Serial.println(counter3);
}

void test(){
  Serial.println("Success");
}