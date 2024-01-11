/*
 *  Homing_controller.cpp
 *
 *  Created on: 11/01/2024
 *      Author: Nopparuj-an
 *
 *  A state-machine based homing controller library.
 *
 *
 * Usage:
 *  ##  Required
 *  --  Optional
 *      Should be called in the order below.
 *
 * ## Include the library:
 *    #include "Homing_controller.h"
 *
 * ## Create an instance of the Homing_controller class:
 *    Homing_controller hc(homing_pin, homing_polarity, timeout, homing_speed);
 *
 * ## Attach motor function:
 *    hc.attachMotor([](float speed){ Mx.set_speed(speed); });
 *
 * -- Attach interrupt to the homing pin (optional):
 *    pinMode(0, INPUT_PULLUP);
 *    attachInterrupt(digitalPinToInterrupt(0), [](){ hc.interruptCallback(); },FALLING);
 *
 * -- Set the brake pin (optional):
 *    hc.setBrakePin(brake_pin, brake_polarity);
 *
 * -- Set the trip current (optional):
 *    hc.setTripCurrent(current);
 *
 * -- Attach complete callback function (may run on interrupt, use with care):
 *    hc.attachCompleteCallback([](){ Mx.encoder_reset(); });
 * 
 * -- Attach fail callback function:
 *    hc.attachFailCallback([](int8_t status){ Serial.print("Homing failed: "); Serial.println(status); });
 *
 * ## Poll for status (REQUIRED for polling mode, timeout and current sensing to work):
 *    int8_t status = hc.poll_for_status();
 *
 * ## Start the homing process:
 *    hc.home();
 *
 * Note:
 * - homing_pin is the interrupt pin number where the homing sensor is connected.
 * - homing_polarity is the state when the sensor is tripped.
 * - timeout is the maximum time (in milliseconds) to wait for the homing process to complete, 0 for no timeout.
 * - brake_pin is the pin number where the brake is connected.
 * - brake_polarity is the signal required to release the brake.
 */

#ifndef HOMING_CONTROLLER_H
#define HOMING_CONTROLLER_H

/* ########## START OF HEADER CONTENT ########## */
#include "Arduino.h"
#include <functional>

class Homing_controller
{
public:
    Homing_controller(int8_t homing_pin, bool homing_polarity, uint32_t timeout, float homing_speed);
    void setBrakePin(int8_t brake_pin, bool brake_polarity);
    void setTripCurrent(float current);
    void home();
    void ARDUINO_ISR_ATTR interruptCallback();
    int8_t poll_for_status(float current = 0);
    void attachMotor(std::function<void(float)> fn);
    void attachCompleteCallback(std::function<void()> fn);
    void attachFailCallback(std::function<void(int8_t)> fn);

private:
    uint32_t timeout_counter; // is set to when timeout is reached
    volatile int status;      // 0: idle, 1: homing, 2: homing done, -1: timeout error, -2: current error

    int8_t homing_pin;    // arduino pin number
    bool homing_polarity; // 0: Home on low, 1: Home on high
    uint32_t timeout;     // in milliseconds, set to 0 for no timeout
    float homing_speed;   // duty cycle or speed
    int8_t brake_pin;     // arduino pin number, set to -1 if not used
    bool brake_polarity;  // 0: Unlock low, 1: Unlock high
    float trip_current;   // in Ampere, set to 0 for no current sensing

    void stop_motor();
    void completeCallback();
    void failCallback(int8_t status);

    std::function<void(float)> fn_motor;
    std::function<void()> complete_callback;
    std::function<void(int8_t)> fail_callback;
};

/* ########## END OF HEADER CONTENT ########## */
#endif