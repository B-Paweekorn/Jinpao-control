#include "Mobile_Config.h"
#include "Wire.h"
#include "esp32-hal.h"
#include "Mobile_command.h"

Mobile_command::Mobile_command(ESP32_CYTRON_MD* _Mx[], QEI* _encx[], PID_CONTROLLER* _pidx[], DC_MOTOR_FFD* _ffdx[], KalmanFilter* _kfx[], Kinematics* _kinematics)
  : kinematics(_kinematics) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    Mx[i] = _Mx[i];
    encx[i] = _encx[i];
    pidx[i] = _pidx[i];
    ffdx[i] = _ffdx[i];
    kfx[i] = _kfx[i];
  }
}

void Mobile_command::begin() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    encx[i]->begin();
  }

  // Wire.begin(ADC_SDA, ADC_SCL, 400000);

  // Wire1.begin(BNO_SDA, BNO_SCL, 400000);
  // while (!bno.begin()) vTaskDelay(10 / portTICK_PERIOD_MS);
  // bno.setExtCrystalUse(true);

  delay(10);

  for (int i = 0; i < NUM_MOTORS; i++) {
    Mx[i]->begin();
    Mx[i]->set_duty(0);

    kfx[i]->begin();

    cmd_ux[i] = 0;
    fb_q[i] = 0;
    fb_qd[i] = 0;
    fb_i[i] = 0;
  }
}

void Mobile_command::control(float _vx, float _vy, float _wz) {
  Kinematics::RadPS wheel_radps = kinematics->Inverse_Kinematics(_vx, _vy, _wz);

  ramp(wheel_radps.radps_fl, 0);
  ramp(wheel_radps.radps_fr, 1);
  ramp(wheel_radps.radps_bl, 2);
  ramp(wheel_radps.radps_br, 3);

  for (int i = 0; i < NUM_MOTORS; i++) {
    fb_q[i] += encx[i]->get_diff_count() * 2 * M_PI / encx[i]->pulse_per_rev;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    float* kf_ptr = kfx[i]->Compute(fb_q[i],
                                    cmd_ux[i] * ffdx[i]->Vmax / ffdx[i]->Umax);
    fb_qd[i] = kf_ptr[1];
    fb_i[i] = kf_ptr[3];

    if (qd_target[i] != 0) {
      cmd_ux[i] = PWM_Satuation(pidx[i]->Compute(qd_target[i] - fb_qd[i]) + ffdx[i]->Compute(qd_target[i], CURRENT_GAIN * fb_i[i]),
                                ffdx[i]->Umax,
                                -1 * ffdx[i]->Umax);
    } else {
      cmd_ux[i] = 0;
    }
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    Mx[i]->set_duty(cmd_ux[i]);
  }
}

IMU_DATA Mobile_command::getIMU() {
  IMU_DATA imu_data;
  sensors_event_t angVelocityData, linearAccelData;

  imu::Quaternion quat = bno.getQuat();
  imu_data.quat.x = quat.x();
  imu_data.quat.y = quat.y();
  imu_data.quat.z = quat.z();
  imu_data.quat.w = quat.w();

  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_data.gyro.x = angVelocityData.gyro.x;
  imu_data.gyro.y = angVelocityData.gyro.y;
  imu_data.gyro.z = angVelocityData.gyro.z;

  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu_data.accel.x = linearAccelData.acceleration.x;
  imu_data.accel.y = linearAccelData.acceleration.y;
  imu_data.accel.z = linearAccelData.acceleration.z;

  return imu_data;
}

ODOM_DATA Mobile_command::getODOM() {
  Kinematics::Velocity robot_odom = kinematics->Forward_Kinematics_Velocity(fb_q[0], fb_q[1], fb_q[2], fb_q[3]);
  ODOM_DATA odom = {
    .vx = robot_odom.vx,
    .vy = robot_odom.vy,
    .wz = robot_odom.wz
  };
  return odom;
}

void Mobile_command::ramp(float set_target, uint8_t index) {
  if (set_target != target[index]) {
    timestamp[index] = millis() + (set_target * 1000.0 / ffdx[index]->qddmax);
    target[index] = set_target;
  }
  if (millis() < timestamp[index]) {
    if (qd_target[index] > target[index]) qd_target[index] -= ffdx[index]->qddmax * dt;
    else if (qd_target[index] < target[index]) qd_target[index] += ffdx[index]->qddmax * dt;
  } else {
    qd_target[index] = target[index];
  }
}
