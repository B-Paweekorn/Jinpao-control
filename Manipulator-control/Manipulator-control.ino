
#include "math.h"

#include "Esp32_Cytron_MDxx.h"

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
int timestep_cmd = 3e6;

float vx, vy, vw = 0;

uint8_t flag = 0;
// Manipulator_command Manipulator(Mx, encx, pidx, ffdx, kfx, kin);
ESP32_CYTRON_MD Mx(39, 40, 1000, 14);



void setup() {
  Serial.begin(115200);

  Mx.begin();

  delay(5000);
}
void loop() {

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();



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
    timestamp = micros();
    //Manipulator.control(vx, vy, vw);
    Mx.set_duty(16383);
  }
}
