#include "CurrentFeedback.h"

#include <Adafruit_ADS1X15.h>
#include <Wire.h>

Adafruit_ADS1115 ads;

CurrentFeedback::CurrentFeedback(int sda, int scl)
  : sda_pin(sda), scl_pin(scl) {}

void CurrentFeedback::begin() {

  Wire.begin(sda_pin, scl_pin);
  Wire.setClock(400000);

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
  } else {
    Serial.println("Pass Initialize ADS.");
  }

  // Setup 3V comparator on channel 0
  ads.startComparator_SingleEnded(0, 1000);
  // Setup 3V comparator on channel 1
  ads.startComparator_SingleEnded(1, 1000);
}

float *CurrentFeedback::read_current() {
  // Comparator will only de-assert after a read

  if (!getOffset) {
    adc0_offset = ads.readADC_SingleEnded(0);
    adc0_offset = ads.readADC_SingleEnded(0);
    voltage0_offset = adc0_offset * ads_sensitivity;

    adc1_offset = ads.readADC_SingleEnded(1);
    adc1_offset = ads.readADC_SingleEnded(1);
    voltage1_offset = adc1_offset * ads_sensitivity;
    getOffset = true;
  }

  adc0 = ads.readADC_SingleEnded(0);
  adc0_Volt = adc0 * ads_sensitivity;
  current[0] = (voltage0_offset - adc0_Volt) / sensitivity;

  adc1 = ads.readADC_SingleEnded(1);
  adc1_Volt = adc1 * ads_sensitivity;
  current[1] = (voltage1_offset - adc1_Volt) / sensitivity;

  return current;
}
