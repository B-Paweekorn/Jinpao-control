/*
 *  Homing_controller.cpp
 *
 *  Created on: 11/01/2024
 *      Author: Nopparuj-an
 *
 *  A state-machine based homing controller library.
 */

#include "Homing_controller.h"

/**
 * @brief Constructs a new Homing Controller object
 *
 * @param homing_pin The pin number where the homing sensor is connected
 * @param homing_polarity The state when the sensor is tripped
 * @param timeout The timeout for the homing controller in milliseconds, set to 0 for no timeout
 * @param homing_speed The speed of homing, can be duty cycle or speed depending on the mode
 */
Homing_controller::Homing_controller(int8_t homing_pin, bool homing_polarity, uint32_t timeout, float homing_speed)
{
    this->timeout_counter = 0;
    this->status = 0;

    this->homing_pin = homing_pin;
    this->homing_polarity = homing_polarity;
    this->timeout = timeout;
    this->homing_speed = homing_speed;
    this->brake_pin = -1;
    this->brake_polarity = 0;
    this->trip_current = 0;

    pinMode(homing_pin, INPUT_PULLUP);
}

/**
 * @brief Constructs a new Homing Controller object using MCP23017
 *
 * @param mcp The MCP23017 object
 * @param homing_pin The pin number where the homing sensor is connected
 * @param homing_polarity The state when the sensor is tripped
 * @param timeout The timeout for the homing controller in milliseconds, set to 0 for no timeout
 * @param homing_speed The speed of homing, can be duty cycle or speed depending on the mode
 */
Homing_controller::Homing_controller(Adafruit_MCP23X17 &mcp, int8_t homing_pin, bool homing_polarity, uint32_t timeout, float homing_speed)
{
    this->timeout_counter = 0;
    this->status = 0;

    this->homing_pin = homing_pin;
    this->homing_polarity = homing_polarity;
    this->timeout = timeout;
    this->homing_speed = homing_speed;
    this->brake_pin = -1;
    this->brake_polarity = 0;
    this->trip_current = 0;
    this->mcp_flag = true;
    this->mcp = &mcp;

    (*this->mcp).pinMode(homing_pin, INPUT_PULLUP);
}

/**
 * @brief Sets the brake pin
 * @note  Do not run this more than once
 *
 * @param brake_pin The brake pin, set to -1 if not used
 * @param brake_polarity The polarity of the brake pin, 0: Unlock low, 1: Unlock high
 */
void Homing_controller::setBrakePin(int8_t brake_pin, bool brake_polarity)
{
    this->brake_pin = brake_pin;
    this->brake_polarity = brake_polarity;

    pinMode(brake_pin, OUTPUT);

    // NOTE: brake pin is a GPIO so it is not necessary to use MCP23017
    // if (this->mcp_flag)
    // {
    //     (*this->mcp).pinMode(brake_pin, OUTPUT);
    // }
    // else
    // {
    //     pinMode(brake_pin, OUTPUT);
    // }
}

/**
 * @brief Sets the trip current
 *
 * @param current The trip current in Ampere
 */
void Homing_controller::setTripCurrent(float current)
{
    this->trip_current = current;
}

/**
 * @brief Starts the homing process
 *
 */
void Homing_controller::home()
{
    // check on entry if the motor is already at home
    bool pin_state = false;
    if (this->mcp_flag)
    {
        pin_state = (*this->mcp).digitalRead(this->homing_pin);
    }
    else
    {
        pin_state = digitalRead(this->homing_pin);
    }

    if (pin_state == this->homing_polarity)
    {
        this->status = 2;

        this->stop_motor();
        this->completeCallback();

        return;
    }

    // set the status to 1, calculate timeout and start the homing process
    this->status = 1;
    this->timeout_counter = millis() + this->timeout;

    // if brake pin is set, release the brake
    if (this->brake_pin != -1)
    {
        if (this->mcp_flag)
        {
            (*this->mcp).digitalWrite(this->brake_pin, this->brake_polarity);
        }
        else
        {
            digitalWrite(this->brake_pin, this->brake_polarity);
        }
    }

    // start the motor
    if (this->fn_motor)
    {
        this->fn_motor(this->homing_speed);
    }
}

/**
 * @brief Returns the status of the homing process
 * @note  Must be called constantly for polling mode, timeout and current sensing to work
 *
 * @param current The current of the motor in Ampere
 *
 * @return int8_t The status of the homing process
 * @retval 0 Idle
 * @retval 1 Homing
 * @retval 2 Homing done
 * @retval -1 Timeout error
 * @retval -2 Current error
 */
int8_t Homing_controller::poll_for_status(float current)
{
    if (this->status == 1)
    {
        // check if the homing process is done by polling (in case the interrupt is missed)
        bool pin_state = false;
        if (this->mcp_flag)
        {
            pin_state = (*this->mcp).digitalRead(this->homing_pin);
        }
        else
        {
            pin_state = digitalRead(this->homing_pin);
        }

        if (pin_state == this->homing_polarity)
        {
            this->status = 2;

            this->stop_motor();
            this->completeCallback();
        }

        // check for timeout
        if (this->timeout != 0 && millis() > this->timeout_counter)
        {
            this->status = -1;
            this->stop_motor();
            this->failCallback(this->status);
        }

        // check for current
        if (this->trip_current != 0 && current > this->trip_current)
        {
            this->status = -2;
            this->stop_motor();
            this->failCallback(this->status);
        }
    }

    // return status 2 and reset status to 0
    if (this->status == 2)
    {
        this->status = 0;
        return 2;
    }

    return this->status;
}

/**
 * @brief The interrupt callback function
 *
 */
void ARDUINO_ISR_ATTR Homing_controller::interruptCallback()
{
    if (this->status != 1)
        return;

    this->stop_motor();
    this->completeCallback();

    this->status = 2;
}

/**
 * @brief Attaches a function to control the motor
 *
 * @param fn The function to be attached
 */
void Homing_controller::attachMotor(std::function<void(float)> fn)
{
    this->fn_motor = fn;
}

/**
 * @brief Attaches a function to the complete callback
 *
 * @param fn The function to be attached
 */
void Homing_controller::attachCompleteCallback(std::function<void()> fn)
{
    this->complete_callback = fn;
}

/**
 * @brief Attaches a function to the fail callback
 *
 * @param fn The function to be attached
 */
void Homing_controller::attachFailCallback(std::function<void(int8_t)> fn)
{
    this->fail_callback = fn;
}

/**
 * @brief Stops the motor
 *
 */
void Homing_controller::stop_motor()
{
    if (this->brake_pin != -1)
    {
        if (this->mcp_flag)
        {
            (*this->mcp).digitalWrite(this->brake_pin, !this->brake_polarity);
        }
        else
        {
            digitalWrite(this->brake_pin, !this->brake_polarity);
        }
    }

    if (this->fn_motor)
    {
        this->fn_motor(0);
    }
}

/**
 * @brief Call complete callback
 *
 */
void Homing_controller::completeCallback()
{
    if (this->complete_callback)
        this->complete_callback();
}

/**
 * @brief Call fail callback
 *
 */
void Homing_controller::failCallback(int8_t status)
{
    if (this->fail_callback)
        this->fail_callback(status);
}