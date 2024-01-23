#include "esp32-hal.h"
#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H
#include "MotionGenerator.h"

MotionGenerator* Trapezoidal_Profile;

float Signal_Generator(uint8_t Type, uint16_t AMP, unsigned long PERIOD_millis) {
  // Current time
  unsigned long current_time = millis();

  // Static variable to remember the last toggle time
  static unsigned long last_toggle_time;
  static float signal;
  static bool signal_on;

  switch (Type) {
    case 0:
      // Check if it's time to toggle the signal state
      if (current_time - last_toggle_time >= PERIOD_millis) {
        signal_on = !signal_on;  // Toggle the state
        last_toggle_time = current_time;
      }
      
      // Set the signal value based on the signal state
      if (signal_on) {
        signal = AMP;
      } else {
        signal = AMP * -1;
      }
      break;

    case 1:
      static int32_t current_pos = 0;

      if (AMP != current_pos) {
        current_pos = Trapezoidal_Profile->update(AMP);
        signal = Trapezoidal_Profile->getVelocity();
      } else if (AMP == current_pos) {
        AMP = AMP * -1;
      }
      break;

    case 2:
      signal = AMP * sin(current_time / 1000.0 / (PERIOD_millis / 1000.0));
      break;
  }


  return signal;
}

#endif