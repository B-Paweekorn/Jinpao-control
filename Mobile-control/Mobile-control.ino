#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Mobile_command.h"
#include "math.h"

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
//Control loop Time (1 Hz)
unsigned long prev_timestep_cmd;
unsigned long current_timestep_cmd;
unsigned long timestamp_cmd = 0;
int timestep_cmd = 2.5e6;

float vx, vy, vw = 0;

uint8_t flag = 0;
Mobile_command Mobile(Mx, encx, pidx, ffdx, kfx, kin);

IMU_DATA imu_data;
ODOM_DATA odom_data;

float dt;

float q_prev[4];

void setup() {
  Serial.begin(115200);
  neopixelWrite(21, 0, 10, 0);

  Mobile.begin();
  neopixelWrite(21, 0, 0, 10);

  delay(5000);
  neopixelWrite(21, 10, 0, 0);
}
void loop() {

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();

    imu_data = Mobile.getIMU();

    // Serial.print(imu_data.accel.x);
    // Serial.print(" ");
    // Serial.print(imu_data.accel.y);
    // Serial.print(" ");
    // Serial.println(imu_data.accel.z);
    // Serial.println(" ");

    // Serial.print(odom_data.vx);
    // Serial.print(" ");
    // Serial.print(odom_data.vy);
    // Serial.print(" ");
    // Serial.print(odom_data.wz);
    // Serial.print(" ");
    Serial.print(dt);
    Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[0]);
    // Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[1]);
    // Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[2]);
    // Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[3]);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[0] - q_prev[0]) * 100);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[1] - q_prev[1]) * 100);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[2] - q_prev[2]) * 100);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[3] - q_prev[3]) * 100);
    Serial.print(Mobile.qd_target[0]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[1]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[2]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[3]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[0]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[1]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[2]);
    Serial.print(" ");
    Serial.println(Mobile.fb_qd[3]);

    q_prev[0] = Mobile.fb_q[0];
    q_prev[1] = Mobile.fb_q[1];
    q_prev[2] = Mobile.fb_q[2];
    q_prev[3] = Mobile.fb_q[3];
  }

  //Cmd loop
  current_timestep_cmd = micros();
  if (current_timestep_cmd - timestamp_cmd > timestep_cmd) {
    timestamp_cmd = micros();
    if (flag == 0) {
      flag = 1;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 1) {
      flag = 2;
      vx = 1;
      vy = 0;
      vw = 0;
    } else if (flag == 2) {
      flag = 3;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 3) {
      flag = 4;
      vx = 0;
      vy = 1;
      vw = 0;
    } else if (flag == 4) {
      flag = 5;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 5) {
      flag = 6;
      vx = 0;
      vy = 0;
      vw = 1;
    } else if (flag == 6) {
      flag = 6;
      vx = 0;
      vy = 0;
      vw = 0;
    }
  }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    dt = current_timestep - timestamp;
    timestamp = micros();
    Mobile.control(vx, vy, vw);

    odom_data = Mobile.getODOM();
  }
}
