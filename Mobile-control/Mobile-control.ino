#include <Mobile_command.h>
#include <Mobile_Config.h>
#include "AS5600.h"
#include "Wire.h"
#include <math.h>

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

Mobile_command Mobile(Mx, encx, pidx, ffdx, kfx, &kinematics);

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  Mobile.begin();
}
void loop() {

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();

    Serial.print(Mobile.fb_qd[0]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[1]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[2]);
    Serial.print(" ");
    Serial.println(Mobile.fb_qd[3]);
  }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    timestamp = micros();
    Mobile.control(0, 0, 0);
  }
}
