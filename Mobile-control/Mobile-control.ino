#include "Mobile_command.h"

<<<<<<< HEAD
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
int timestep_cmd = 2.5e6;
=======
void timer_callback();
void control_task_callback(void *pv);
>>>>>>> 53e2cae07b648baf61b2ca8cfb40baa61d800e8e

TaskHandle_t control_task = NULL;
hw_timer_t *timer = NULL;

uint64_t timestamp_cmd = 0;
int timestep_cmd = 2.5e6;

uint64_t timestamp_print = 0;
int timestep_print = 10e3;

float vx[7] = { 0, 1, 0, 0, 0, 0, 0 };
float vy[7] = { 0, 0, 0, 1, 0, 0, 0 };
float vw[7] = { 0, 0, 0, 0, 0, 1, 0 };

uint8_t flag = 0;
Mobile_command Mobile(Mx, encx, pidx, ffdx, kfx, kin);

IMU_DATA imu_data;
ODOM_DATA odom_data;

float dt;

float q_prev[4];

void setup() {
  Serial.begin(115200);
  neopixelWrite(21, 0, 10, 0);

  Mobile.begin();
  neopixelWrite(21, 0, 0, 10);

  delay(1000);
  xTaskCreatePinnedToCore(control_task_callback,
                          "control-task",
                          8192,
                          NULL,
                          15,
                          &control_task,
                          1);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &timer_callback);
  timerAlarm(timer, 1000, true, 0);

  timestamp_cmd = micros();
  timestamp_print = micros();

  neopixelWrite(21, 10, 0, 0);
}

<<<<<<< HEAD
<<<<<<< HEAD
  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp >= timestep) {
    dt = current_timestep - timestamp;
    timestamp = micros();
    Mobile.control(vx, vy, vw);

    odom_data = Mobile.getODOM();
  }

=======
>>>>>>> parent of e3eaff8 (last commit)
  //Print loop
  current_timestep_print = micros();
  if (current_timestep_print - timestamp_print > timestep_print) {
    timestamp_print = micros();

    imu_data = Mobile.getIMU();

    // Serial.print(imu_data.accel.x);
    // Serial.print(" ");
    // Serial.print(imu_data.accel.y);
    // Serial.print(" ");
    // Serial.println(imu_data.accel.z);
    // Serial.println(" ");

    // Serial.print(odom_data.vx);
    // Serial.print(" ");
    // Serial.print(odom_data.vy);
    // Serial.print(" ");
    // Serial.print(odom_data.wz);
    // Serial.print(" ");
    Serial.print(dt);
    Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[0]);
    // Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[1]);
    // Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[2]);
    // Serial.print(" ");
    // Serial.print(Mobile.cmd_ux[3]);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[0] - q_prev[0]) * 100);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[1] - q_prev[1]) * 100);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[2] - q_prev[2]) * 100);
    // Serial.print(" ");
    // Serial.print((Mobile.fb_q[3] - q_prev[3]) * 100);
<<<<<<< HEAD
    // Serial.print(Mobile.qd_target[0]);
    // Serial.print(" ");
    // Serial.print(Mobile.qd_target[1]);
    // Serial.print(" ");
    // Serial.print(Mobile.qd_target[2]);
    // Serial.print(" ");
    // Serial.print(Mobile.qd_target[3]);
    // Serial.print(" ");
=======
void IRAM_ATTR timer_callback() {
  BaseType_t task_woken;
  task_woken = xTaskResumeFromISR(control_task);
  portYIELD_FROM_ISR(task_woken);
}

void control_task_callback(void *pv) {
  while (true) {
    vTaskSuspend(NULL);
    Mobile.control(vx[flag], vy[flag], vw[flag]);
  }
}

void loop() {
  uint64_t time = micros();
  if (time - timestamp_cmd > timestep_cmd) {
    timestamp_cmd = time;
    if (flag < 6) flag++;
  }

  if (time - timestamp_print > timestep_print) {
    timestamp_print = time;

    imu_data = Mobile.getIMU();
    odom_data = Mobile.getODOM();
    
=======
>>>>>>> parent of e3eaff8 (last commit)
    Serial.print(Mobile.qd_target[0]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[1]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[2]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[3]);
    Serial.print(" ");
<<<<<<< HEAD
>>>>>>> 53e2cae07b648baf61b2ca8cfb40baa61d800e8e
=======
>>>>>>> parent of e3eaff8 (last commit)
    Serial.print(Mobile.fb_qd[0]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[1]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[2]);
    Serial.print(" ");
    Serial.println(Mobile.fb_qd[3]);

    q_prev[0] = Mobile.fb_q[0];
    q_prev[1] = Mobile.fb_q[1];
    q_prev[2] = Mobile.fb_q[2];
    q_prev[3] = Mobile.fb_q[3];
  }
<<<<<<< HEAD

  //Cmd loop
  current_timestep_cmd = micros();
  if (current_timestep_cmd - timestamp_cmd > timestep_cmd) {
    timestamp_cmd = micros();
    if (flag == 0) {
      flag = 1;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 1) {
      flag = 2;
      vx = 1;
      vy = 0;
      vw = 0;
    } else if (flag == 2) {
      flag = 3;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 3) {
      flag = 4;
      vx = 0;
      vy = 1;
      vw = 0;
    } else if (flag == 4) {
      flag = 5;
      vx = 0;
      vy = 0;
      vw = 0;
    } else if (flag == 5) {
      flag = 6;
      vx = 0;
      vy = 0;
      vw = 1;
    } else if (flag == 6) {
      flag = 6;
      vx = 0;
      vy = 0;
      vw = 0;
    }
  }
<<<<<<< HEAD
=======
>>>>>>> 53e2cae07b648baf61b2ca8cfb40baa61d800e8e
=======

  //Control loop
  current_timestep = micros();
  if (current_timestep - timestamp > timestep) {
    dt = current_timestep - timestamp;
    timestamp = micros();
    Mobile.control(vx, vy, vw);

    odom_data = Mobile.getODOM();
  }
>>>>>>> parent of e3eaff8 (last commit)
}
