
#include "math.h"
#include "Gripper_command.h"
#include "Esp32_Cytron_MDxx.h"
#include "SignalGenerator.h"
//#include "MotionGenerator.h"
//Print loop Time (100 Hz)
unsigned long prev_timestep_print;
unsigned long current_timestep_print;
unsigned long timestamp_print = 0;
int32_t timestep_print = 100000;
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
Gripper_command Gripper(Mx, encx, pidx_pos, pidx_vel, ffdx, kfx, tpx);

//float signal = 0;
float q_prev = 0;

double newVt[5] = { 0, 8192, 8192, 8192, 8192 };
uint8_t i = 0;
//Generate Wave Form
// uint8_t signal_form = 2;
uint16_t Amplitude = 11;
unsigned long period_milli = 5000;

//Tunning State
uint8_t state = 'K';  // 'K':Kalman, 'V':Velocity, 'P':Position

unsigned long startTime = 0;  // Variable to store the start time
float signal = 0;             // Variable to store the signal value
//uint16_t Amplitude = 30;      // Maximum amplitude of the waveform

// Durations for each phase in milliseconds
unsigned long rampUpTime = 5000;
unsigned long highTime = 1000;
unsigned long rampDownTime = 5000;
unsigned long lowTime = 1000;

// Total period of the waveform
unsigned long totalPeriod = rampUpTime + highTime + rampDownTime + lowTime;
uint32_t pos = 0;

void setup() {
  Serial.begin(115200);
  Gripper.begin();
  delay(5000);
}
void loop() {
  if (tpx[0]->getFinished()) {
    if (i < 4) {
      i += 1;
    } else {
      i = 3;
    }
  }

  unsigned long currentTime = micros();
  unsigned long elapsedTime = (currentTime - startTime) / 1000;  // Elapsed time in milliseconds
  unsigned long timeInPeriod = elapsedTime % totalPeriod;        // Current time within the waveform period


  if (timeInPeriod < rampUpTime) {
    // Ramp-up phase
    signal = (float)Amplitude * timeInPeriod / rampUpTime;
  } else if (timeInPeriod < rampUpTime + highTime) {
    // High level phase
    signal = Amplitude;
  } else if (timeInPeriod < rampUpTime + highTime + rampDownTime) {
    // Ramp-down phase
    unsigned long timeInRampDown = timeInPeriod - (rampUpTime + highTime);
    signal = (float)Amplitude * (1 - (float)timeInRampDown / rampDownTime);
  } else {
    // Low level phase
    signal = 0;
  }

  // signal = 12;
  // signal = Signal_Generator(2, Amplitude, period_milli);  //wave form, Amplitude, period (ms)

  // if ((USBSerial.available() > 0) && (state == 'P')) {
  //   // read the incoming byte:
  //   String USBSerialInput = USBSerial.readString();
  //   newVt = USBSerialInput.toDouble();
  //   // say what you got:
  //   USBSerial.print("I received: ");
  //   USBSerial.println(USBSerialInput);
  // }

  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();
    // Serial.print(signal);
    // Serial.print(" ");
    // Serial.println();
    //Serial.print(signal);
    // Serial.print(" ");

    Serial.print(Gripper.fb_q[0]);
    Serial.print(" ");
    Serial.println(tpx[0]->getVelocity());

    // Serial.print(Gripper.q_target[0], 10);
    // Serial.print(" ");
    // Serial.print(Gripper.fb_q[0], 10);
    // Serial.print(" ");
    // Serial.println(Gripper.q_target[0] - Gripper.fb_q[0], 10);

    // Serial.print(" ");
    // Serial.print(-10);
    // Serial.print(" ");
    // Serial.println(10);
    // Serial.print(" ");

    // Serial.print(" ");
    // Serial.print(Gripper.q_target[0] * 3072 / (2 * M_PI), 10);
    // Serial.print(" ");
    // Serial.print(Gripper.fb_q_pulse[0], 10);
    // Serial.print(" ");
    // Serial.print(Gripper.fb_q_pulse[0] - q_prev, 10);

    // Serial.print(" ");
    // Serial.print(Gripper.cmd_vx[0]);

    // Serial.print(" ");
    // Serial.print(Gripper.qd_target[0], 10);
    // Serial.print(" ");
    // Serial.print(Gripper.fb_qd[0], 10);
    // Serial.print(" ");
    // Serial.println(Gripper.qd_target[0] - Gripper.fb_qd[0], 10);


    // Serial.print(" ");
    // Serial.println(Gripper.fb_i[0]);
    // Serial.print(" ");
    // Serial.println(Gripper.cmd_vx[0]);
    // Serial.print(" ");
    // Serial.print((Gripper.fb_q[0] - q_prev) * 100.0);  //Gripper.qd_target[3]
    // Serial.print(" ");
    // Serial.println(Gripper.fb_qd[0]);
    // q_prev = Gripper.fb_q_pulse[0];
  }

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    timestamp = micros();
    // Mx[0]->set_duty(6 * 16383 / 12.0);
    //Mx[0]->set_duty(signal * 16383 / 12.0);


    // //Kalman Filter Tunning
    //Gripper.tune(0, signal, state);

    // //Velocity Control Tunning
    // if(state == 'V') Gripper.tune(3, signal, state);

    // //Position Control Tunning
    // if(state == 'P')
    // Gripper.setGoal(0, pos);
    Gripper.setGoal(0, newVt[i]);
  }
}