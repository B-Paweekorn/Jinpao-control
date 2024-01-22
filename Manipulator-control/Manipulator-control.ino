
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
}
void loop() {
  // signal = Signal_Generator(2, 12, 5000);  //wave form, Amplitude, period (ms)

  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   String serialInput = Serial.readString();
  //   newVt = serialInput.toDouble();
  //   // say what you got:
  //   Serial.print("I received: ");
  //   Serial.println(serialInput);
  // }

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();
    // counter0 += (encx[1]->get_diff_count());
    // Serial.print(counter0);
    // Serial.print(" ");
    // Serial.print(encx[0]->get_diff_count());
    // Serial.print(" ");
    Serial.print(Manipulator.fb_q[0]);
    Serial.print(" ");
    Serial.print(Manipulator.q_target[0]);
    Serial.print(" ");
    Serial.println(Manipulator.cmd_ux[0]);
    Serial.print(" ");
    // Serial.println(Manipulator.fb_qd[0]);
    // Serial.print(" ");
    // Serial.println(Manipulator.qd_target[0]);
    // Serial.print(" ");
    // Serial.println(Manipulator.fb_qd[0]);
    // q_prev = Manipulator.fb_q[0];
  }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    timestamp = micros();
    // Manipulator.tune(1, signal);
    // signal= 3276;
    // Mx[0]->set_duty(signal);
    // Mx[1]->set_duty(signal);
    // Mx[2]->set_duty(signal);
    // Mx[3]->set_duty(signal);
    // //Manipulator.tune(1, );
    Manipulator.setGoal(0, 15);
  }


  //Signal generator loop
  // current_timestep_signal = micros();
  // if (current_timestep_signal - timestamp_signal > timestep_signal) {
  //   timestamp_signal = micros();
  //   if (signal < 3276) { signal += 1; }
  //   if (signal == 3276) { signal = 0; }
  // }
}
