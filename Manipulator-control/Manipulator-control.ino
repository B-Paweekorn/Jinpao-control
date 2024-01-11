
#include "math.h"
#include "Manipulator_command.h"
#include "Esp32_Cytron_MDxx.h"
#include "SignalGenerator.h"

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
Manipulator_command Manipulator(Mx, encx, pidx_pos, pidx_vel, ffdx, kfx, tpx);

float signal = 0;
float q_prev = 0;

double newVt;


void setup() {
  Serial.begin(115200);
  Manipulator.begin();


  delay(5000);
}
void loop() {
  //signal = Signal_Generator(2, 11.8, 500);  //Sine Wave

  if (Serial.available() > 0) {
    // read the incoming byte:
    String serialInput = Serial.readString();
    newVt = serialInput.toDouble();
    // say what you got:
    Serial.print("I received: ");
    Serial.println(serialInput);
  }

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();
    // Serial.print(signal);
    // Serial.print(" ");
    Serial.print(Manipulator.fb_q[0]);
    Serial.print(" ");
    Serial.print(Manipulator.q_target[0]);
    Serial.print(" ");
    Serial.print(Manipulator.fb_qd[0]);
    Serial.print(" ");
    Serial.println(Manipulator.qd_target[0]);
    // Serial.print((Manipulator.fb_q[0] - q_prev) * 100);
    // Serial.print(" ");
    // Serial.println(Manipulator.fb_qd[0]);
    q_prev = Manipulator.fb_q[0];
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
    //Manipulator.tune(0, signal);
    //Manipulator.tune(1, );
    Manipulator.setGoal(0, newVt);
  }


  //Signal generator loop
  // current_timestep = micros();
  // if (current_timestep - timestamp > timestep) {
  //   timestamp = micros();
  // signal = ???

  // }
}
