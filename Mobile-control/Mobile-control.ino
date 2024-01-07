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

unsigned long tcur;
/*-----QEI(encA, encB)------*/

QEI enc1(17, 18);
QEI enc2(33, 34);
QEI enc3(35, 36);
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
PID pid1(&MOTOR_1, &enc1, &kf1, 10, 0.0001f, 0.0f);
PID pid2(&MOTOR_2, &enc2, &kf2, 500, 0.0f, 0.0f);
PID pid3(&MOTOR_3, &enc3, &kf3, 500, 0.0f, 0.0f);
PID pid4(&MOTOR_4, &enc4, &kf4, 500, 0.0f, 0.0f);

/*-----Config Robot Base-----*/
#define WHEEL_DIAMETER 0.1524  //meter
#define LX 0.2105              //meter
#define LY 0.31                //meter

Kinematics kinematics(WHEEL_DIAMETER, LX, LY);


/*-----Pass value-----*/
setMotor* motors[4] = { &MOTOR_1, &MOTOR_2, &MOTOR_3, &MOTOR_4 };
QEI* encoders[4] = { &enc1, &enc2, &enc3, &enc4 };
PID* pids[4] = { &pid1, &pid2, &pid3, &pid4 };

Mobile_command Mobile(motors, encoders, pids, &kinematics);

float counts_per_rev = 2048 * 4.0;
int32_t pos_prev = 0;

double q = 0;
double q_prev = 0;

float v_est;
float u_volt;


void setup() {
  Serial.begin(115200);

  /*-----Setup Hardware Start-----*/

  // ledcAttach(MOTOR_1_PWM_PIN, MOTOR_BASE_FREQ, MOTOR_TIMER_14_BIT);
  // pinMode(MOTOR_1_DIR_PIN, OUTPUT);

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

  // pid1.setK(1, 1, 1);

  /*-----Setup Hardware End-------*/
}

//Generate Sinwave
float dummy_sinwave = 0;
#define SIMUL_PERIOD 1     // oscillating period [s]
#define SIMUL_AMP 16383.0  // oscillation amplitude
void loop() {

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    // Serial.print(current_timestep_print - timestamp_print);
    // Serial.print(" ");
    Serial.print(u_volt);
    Serial.print(" ");
    Serial.print((q - q_prev) * 100);
    Serial.print(" ");
    Serial.println(v_est);
    q_prev = q;
    timestamp_print = micros();


    // Serial.print(pid1.targetRads);
    // Serial.print(" ");
    // Serial.println(pid1.v);
    // Serial.print(" ");
    // Serial.print(pid1.e);
    // Serial.print(" ");
    // Serial.print(pid1.u);
    // Serial.print(" ");
    // Serial.println(pid1.PWM_feedforward);


    // uint32_t time = micros();

    // counter0 += enc1.get_diff_count();
    // counter1 += enc2.get_diff_count();
    // counter2 += enc3.get_diff_count();
    // counter3 += enc4.get_diff_count();

    // uint32_t dt = micros() - time;

    // Serial.print(dt);
    // Serial.print(' ');
    // Serial.print(counter0);
    // Serial.print(' ');
    // Serial.print(counter1);
    // Serial.print(' ');
    // Serial.print(counter2);
    // Serial.print(' ');
    // Serial.println(counter3);
  }
  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    // Serial.println(current_timestep - timestamp);
    timestamp = micros();
    tcur++;
    dummy_sinwave = SIMUL_AMP * sin(tcur / 1000.0 / SIMUL_PERIOD);
    float u = dummy_sinwave;
    float Vin = u * 18.0 / 16383;
    // Mobile.control(5, 0, 0);
    // pid1.setRads(dummy_sinwave);
    // pid1.setRads(2 * M_PI);
    // pid1.compute();

    // int pos = enc1.get_diff_count();
    // Serial.println(pos - pos_prev);
    float dq = (enc1.get_diff_count() * 2 * M_PI) / counts_per_rev;
    q += dq;
    v_est = kf1.EstimateSpeed(q, Vin);
    MOTOR_1.setPWM(u);
    u_volt = u / 16383.0 * 18.0;

    //Serial.println(current_timestep - timestamp);
  }




  /*-----Test PWM Start (0 - 16383)-----*/
  // %duty 0 - 16382
  // MOTOR_1.setPWM(16383);
  // MOTOR_2.setPWM(8192);
  // MOTOR_3.setPWM(12288);
  // MOTOR_4.setPWM(16383);

  /*-----Test PWM End-------*/
}

void test() {
  Serial.println("Success");
}
