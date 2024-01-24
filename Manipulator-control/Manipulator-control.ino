
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
//Signal loop Time (1 Hz)
unsigned long prev_timestep_signal;
unsigned long current_timestep_signal;
unsigned long timestamp_signal = 0;
int timestep_signal = 611;

float vx, vy, vw = 0;

uint8_t flag = 0;
Manipulator_command Manipulator(Mx, encx, pidx_pos, pidx_vel, ffdx, kfx, tpx, hcx, mcp);

float signal = 0;
float q_prev = 0;

double newVt;

int32_t counter0 = 0;
void setup() {
  Serial.begin(115200);

  Manipulator.begin();
  // Manipulator.setHomeAll();
  // Manipulator.pollHoming();

  delay(5000);
  Manipulator.setGoal(0, 0);
  Manipulator.setGoal(1, 0);
  Manipulator.setGoal(2, 0);
  Manipulator.setGoal(3, 0);
}
void loop() {
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();
    Serial.print(Manipulator.fb_q[0]);
    Serial.print(" ");
    Serial.print(Manipulator.fb_q[1]);
    Serial.print(" ");
    Serial.print(Manipulator.fb_q[2]);
    Serial.print(" ");
    Serial.print(Manipulator.fb_q[3]);
    Serial.print(" ");
    Serial.print(Manipulator.q_target[0]);
    Serial.print(" ");
    Serial.print(Manipulator.q_target[1]);
    Serial.print(" ");
    Serial.print(Manipulator.q_target[2]);
    Serial.print(" ");
    Serial.println(Manipulator.q_target[3]);
  }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    timestamp = micros();
    Manipulator.setGoal(0, 15); // max 30
    Manipulator.setGoal(1, 1); // max 5
    Manipulator.setGoal(2, 3); // max 5
    Manipulator.setGoal(3, 5); // max 5
  }
}
