#include "esp32-hal.h"
#ifndef MOBILE_COMMAND_H
#define MOBILE_COMMAND_H

#include <Arduino.h>

// #include <QEI.h>
// #include <Kinematics.h>
// #include <KalmanFilter.h>
// #include <Controller.h>
// #include <Esp32_Cytron_MDxx.h>
#include <Mobile_Config.h>
#include "Wire.h"

#define CURRENT_GAIN 18.0 / 26.6

class Mobile_command {
private:
    static const int NUM_MOTORS = 4; // Number of motors, encoders, and PIDs
    float dt = 1 / 1000.0;

    ESP32_CYTRON_MD* Mx[NUM_MOTORS];
    QEI* encx[NUM_MOTORS];
    PID_CONTROLLER* pidx[NUM_MOTORS];
    DC_MOTOR_FFD* ffdx[NUM_MOTORS];
    KalmanFilter* kfx[NUM_MOTORS];
    Kinematics* kinematics;    

    uint32_t timestamp[NUM_MOTORS];
    float target[NUM_MOTORS];
    float qd_target[NUM_MOTORS]; 

    void ramp(float set_target, uint8_t index);

public:

    int16_t cmd_ux[NUM_MOTORS];
    double fb_q[NUM_MOTORS];
    float fb_qd[NUM_MOTORS];
    float fb_i[NUM_MOTORS];

    Mobile_command(ESP32_CYTRON_MD* _Mx[], QEI* _encx[], PID_CONTROLLER* _pidx[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], Kinematics* _kinematics);

    void begin();

    void control(float _vx, float _vy, float _wz);

};

#endif
