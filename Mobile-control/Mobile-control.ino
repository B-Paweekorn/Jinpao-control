#include "Mobile_command.h"

void IRAM_ATTR timer_callback();
void control_task_callback(void *pv);

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
    
    Serial.print(Mobile.qd_target[0]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[1]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[2]);
    Serial.print(" ");
    Serial.print(Mobile.qd_target[3]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[0]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[1]);
    Serial.print(" ");
    Serial.print(Mobile.fb_qd[2]);
    Serial.print(" ");
    Serial.println(Mobile.fb_qd[3]);
  }
}
