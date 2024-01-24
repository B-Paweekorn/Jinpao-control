
#include "math.h"
#include "Manipulator_command.h"
#include "Esp32_Cytron_MDxx.h"
#include "SignalGenerator.h"
#include <Adafruit_MCP23X17.h>

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
Manipulator_command Manipulator(Mx, encx, pidx_pos, pidx_vel, ffdx, kfx, tpx);

float signal = 0;
float q_prev = 0;

double newVt;
int32_t counter0 = 0;

Adafruit_MCP23X17 mcp;

void setup() {
  Serial.begin(115200);

  Wire.begin(MCP_SDA, MCP_SCL);

  if (!mcp.begin_I2C()) {
    Serial.println("################ MCP Error ################");
    delay(1000);
    ESP.restart();
  } else {
    Serial.println("MCP Init OK");
  }

  mcp.pinMode(0, OUTPUT);       // GPA0: OUTPUT
  mcp.pinMode(1, OUTPUT);       // GPA1: OUTPUT
  mcp.pinMode(2, OUTPUT);       // GPA2: OUTPUT

  Manipulator.begin();

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

void magnet(uint8_t ID, bool state) {
  uint8_t pins[3] = {0, 1, 2};
  mcp.digitalWrite(pins[ID], state);
}
