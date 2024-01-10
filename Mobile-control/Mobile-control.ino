#include "QEI.h"
#include "setMotor.h"
#include "Kinematics.h"
#include "PID.h"
#include "Mobile_command.h"
#include "AS5600.h"
#include "Wire.h"
#include "KalmanFilter.h"
#include <math.h>
#include "Cytron_Motor_260rpm_250W.h"

#define I2C_SDA 13
#define I2C_SCL 14

//Print loop Time (100 Hz)
unsigned long prev_timestep_print;
unsigned long current_timestep_print;
unsigned long timestamp_print = 0;
int32_t timestep_print = 10000;
//Control loop Time (1000 Hz)
unsigned long prev_timestep;
unsigned long current_timestep;
unsigned long timestamp = 0;
int timestep = 1000;

uint32_t timestamp_update = 0;

unsigned long tcur;
/*-----QEI(encA, encB)------*/

QEI enc1(18, 17);
QEI enc2(33, 34);
QEI enc3(36, 35);
QEI enc4(37, 38);

long counter0 = 0;
long counter1 = 0;
long counter2 = 0;
long counter3 = 0;

/*-----Config Motor -----*/

#define MOTOR_TIMER_14_BIT 14
#define MOTOR_BASE_FREQ 1000

#define MOTOR_1_PWM_PIN 39
#define MOTOR_2_PWM_PIN 42
#define MOTOR_3_PWM_PIN 1
#define MOTOR_4_PWM_PIN 4

#define MOTOR_1_PWM_CHANNEL 0
#define MOTOR_2_PWM_CHANNEL 1
#define MOTOR_3_PWM_CHANNEL 2
#define MOTOR_4_PWM_CHANNEL 3

#define MOTOR_1_DIR_PIN 40
#define MOTOR_2_DIR_PIN 41
#define MOTOR_3_DIR_PIN 2
#define MOTOR_4_DIR_PIN 5

setMotor MOTOR_1(MOTOR_1_PWM_PIN, MOTOR_1_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
setMotor MOTOR_2(MOTOR_2_PWM_PIN, MOTOR_2_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
setMotor MOTOR_3(MOTOR_3_PWM_PIN, MOTOR_3_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
setMotor MOTOR_4(MOTOR_4_PWM_PIN, MOTOR_4_DIR_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);

/*-----Config ADC-----*/
#define ADC_SDA 13
#define ADC_SCL 14

/*-----Config BNO-----*/
#define BNO_SDA 15
#define BNO_SCL 16

/*-----Config Kalman -----*/
KalmanFilter kf1(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);
KalmanFilter kf2(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);
KalmanFilter kf3(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);
KalmanFilter kf4(CYTRON_MOTOR_260RPM_250W_MatrixA,
                 CYTRON_MOTOR_260RPM_250W_MatrixB,
                 CYTRON_MOTOR_260RPM_250W_MatrixC,
                 CYTRON_MOTOR_260RPM_250W_MatrixQ,
                 CYTRON_MOTOR_260RPM_250W_MatrixR);

int32_t rawAngle_prev, count;

/*-----Config PID -----*/
PID pid1(&MOTOR_1, &enc1, &kf1, 7.5, 7.5, 0.0);
PID pid2(&MOTOR_2, &enc2, &kf2, 7.5, 7.5, 0.0);
PID pid3(&MOTOR_3, &enc3, &kf3, 7.5, 7.5, 0.0);
PID pid4(&MOTOR_4, &enc4, &kf4, 7.5, 7.5, 0.0);

/*-----Config Robot Base-----*/
#define WHEEL_DIAMETER 0.1524  //meter
#define LX 0.2105              //meter
#define LY 0.31                //meter

Kinematics kinematics(WHEEL_DIAMETER, LX, LY);

/*-----Pass value-----*/
setMotor *motors[4] = { &MOTOR_1, &MOTOR_2, &MOTOR_3, &MOTOR_4 };
QEI *encoders[4] = { &enc1, &enc2, &enc3, &enc4 };
PID *pids[4] = { &pid1, &pid2, &pid3, &pid4 };

Mobile_command Mobile(motors, encoders, pids, &kinematics);

float counts_per_rev = 2048 * 4.0;
int32_t pos_prev = 0;

double q = 0;
double q_prev = 0;

float v_est;
float u_volt;

float set_point = 0;

struct RadPs {
  float radps_fl;
  float radps_fr;
  float radps_bl;
  float radps_br;
};
struct RadPs wheel_radps;
void setup() {
  Serial.begin(230400);

  /*-----Setup Hardware Start-----*/
  enc1.begin();
  enc2.begin();
  enc3.begin();
  enc4.begin();
  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize I2C communication


  MOTOR_1.begin();
  MOTOR_2.begin();
  MOTOR_3.begin();
  MOTOR_4.begin();

  kf1.init();
  kf2.init();
  kf3.init();
  kf4.init();

  MOTOR_1.setPWM(0);
  MOTOR_2.setPWM(0);
  MOTOR_3.setPWM(0);
  MOTOR_4.setPWM(0);
  delay(5000);

  timestamp_update = micros();
}

//Generate Sinwave
float dummy_sinwave = 0;
#define SIMUL_PERIOD 1     // oscillating period [s]
#define SIMUL_AMP 16383.0  // oscillation amplitude

float *kf1_ptr;
float *kf2_ptr;
float *kf3_ptr;
float *kf4_ptr;

double q1, q2, q3, q4 = 0;
float qd1, qd2, qd3, qd4;
float i1, i2, i3, i4;

uint8_t flag = 0;
void loop() {
  Serial.print(flag);
  Serial.print(" ");
  Serial.print(pid1.targetRads);
  Serial.print(" ");
  Serial.print(pid2.targetRads);
  Serial.print(" ");
  Serial.print(pid3.targetRads);
  Serial.print(" ");
  Serial.println(pid4.targetRads);
  // current_timestep_print = micros();
  // if (current_timestep_print - timestamp_print > timestep_print) {
  //   Serial.print(pid1.v);
  //   Serial.print(" ");
  //   Serial.print(pid2.v);
  //   Serial.print(" ");
  //   Serial.print(pid3.v);
  //   Serial.print(" ");
  //   Serial.println(pid4.v);
  // }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    timestamp = micros();
    if(flag == 0){
      Mobile.control(1, 0, 0);
    }
    else if(flag == 1){
      Mobile.control(0, 0, 0);
    }
    else if(flag == 2){
      Mobile.control(0, 1, 0);
    }
    else if(flag == 3){
      Mobile.control(0, 0, 0);
    }
    else if(flag == 4){
      Mobile.control(0, 0, 1);
    }
    else if(flag == 5){
      Mobile.control(0, 0, 0);
    }
  }

  if (current_timestep - timestamp_update > 2e6) {
    timestamp_update = micros();
    if(flag == 0){
      flag = 1;
    }
    else if(flag == 1){
      flag = 2;
    }
    else if(flag == 2){
      flag = 3;
    }
    else if(flag == 3){
      flag = 4;
    }
    else if(flag == 4){
      flag = 5;
    }
  }
}

void test() {
  Serial.println("Success");
}
