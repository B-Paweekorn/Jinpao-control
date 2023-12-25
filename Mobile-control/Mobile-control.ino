#include "QEI.h"

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

/*-----Config ADC-----*/
#define ADC_SDA 13 
#define ADC_SCL 14

/*-----Config BNO-----*/
#define BNO_SDA 15
#define BNO_SCL 16

void setup() {
  Serial.begin(115200);

  /*-----Setup Motor Start-----*/

  ledcAttach(MOTOR_1_PWM_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);
  ledcAttach(MOTOR_2_PWM_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);
  ledcAttach(MOTOR_3_PWM_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);
  ledcAttach(MOTOR_4_PWM_PIN, MOTOR_BASE_FREQ , MOTOR_TIMER_14_BIT);

  pinMode(MOTOR_1_DIR_PIN, OUTPUT);
  pinMode(MOTOR_2_DIR_PIN, OUTPUT);
  pinMode(MOTOR_3_DIR_PIN, OUTPUT);
  pinMode(MOTOR_4_DIR_PIN, OUTPUT);

  /*-----Setup Motor End-----*/

  /*-----Setup QEI Start-----*/

  enc0.begin();
  enc1.begin();
  enc2.begin();
  enc3.begin();

  /*-----Setup QEI End-------*/
}

void loop() {
  /*-----Test PWM Start-----*/

  ledcWrite(MOTOR_1_PWM_PIN, 4096);
  ledcWrite(MOTOR_2_PWM_PIN, 8192);
  ledcWrite(MOTOR_3_PWM_PIN, 12288);
  ledcWrite(MOTOR_4_PWM_PIN, 16383);

  /*-----Test PWM End-------*/

  uint32_t time = micros();

  counter0 += enc0.get_diff_count();
  counter1 += enc1.get_diff_count();
  counter2 += enc2.get_diff_count();
  counter3 += enc3.get_diff_count();

  // %duty 0 - 16382

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