#include "Mobile_command.h"
#include "MCU_msgs.h"

void timer0_callback();
void control_loop(void *pv);

Mobile_command Mobile(Mx, encx, pidx, ffdx, kfx, kin);

IMU_DATA imu_data;
ODOM_DATA odom_data;
ODOM_DATA odom_cmd;

TaskHandle_t control_task;
SemaphoreHandle_t control_sem;
SemaphoreHandle_t update_sem;
hw_timer_t *htim0;

MCU_msgs esp_msgs;
MCU_msgs ser_msgs;

uint8_t tx_msg_buffer[120];

void setup() {
  Serial.begin(460800);
  Serial.setTimeout(1);
  while (!Serial)
    ;
  neopixelWrite(21, 0, 10, 0);

  Mobile.begin();
  MCU_msgs_Init(&esp_msgs,
                13,
                77,  // M
                2);
  MCU_msgs_Init(&ser_msgs,
                3,
                83,  // S
                2);
  neopixelWrite(21, 0, 0, 10);

  delay(5000);
  control_sem = xSemaphoreCreateBinary();
  update_sem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(control_loop,
                          "control-task-loop",
                          8192,
                          NULL,
                          tskIDLE_PRIORITY,
                          &control_task,
                          0);

  htim0 = timerBegin(1000000);
  timerAttachInterrupt(htim0, &timer0_callback);
  timerAlarm(htim0, 1000, true, 0);
  neopixelWrite(21, 10, 0, 0);
}

void timer0_callback() {
  BaseType_t task_woken;
  xSemaphoreGiveFromISR(control_sem, &task_woken);
  portYIELD_FROM_ISR(task_woken);
}

void control_loop(void *pv) {
  while (true) {
    static int counter = 0;

    if (Serial.available()) {
      decode_buffer2msg(&ser_msgs, Serial.read());
      if (ser_msgs.status) {
        odom_cmd.vx = ser_msgs.buff[0].f64;
        odom_cmd.vy = ser_msgs.buff[1].f64;
        odom_cmd.wz = ser_msgs.buff[2].f64;
      }
    }

    if (xSemaphoreTake(control_sem, 0) == pdTRUE) {
      Mobile.control(odom_cmd.vx, odom_cmd.vy, odom_cmd.wz);
      counter = (counter + 1) % 10;
      if (!counter) xSemaphoreGive(update_sem);
    }
  }
}

void loop() {
  if (xSemaphoreTake(update_sem, 0) == pdTRUE) {
    // imu_data = Mobile.getIMU();
    odom_data = Mobile.getODOM();

    esp_msgs.buff[0].f64 = odom_data.vx;
    esp_msgs.buff[1].f64 = odom_data.vy;
    esp_msgs.buff[2].f64 = odom_data.wz;

    esp_msgs.buff[3].f64 = imu_data.gyro.x;
    esp_msgs.buff[4].f64 = imu_data.gyro.y;
    esp_msgs.buff[5].f64 = imu_data.gyro.z;

    esp_msgs.buff[6].f64 = imu_data.accel.x;
    esp_msgs.buff[7].f64 = imu_data.accel.y;
    esp_msgs.buff[8].f64 = imu_data.accel.z;

    esp_msgs.buff[9].f64 = imu_data.quat.x;
    esp_msgs.buff[10].f64 = imu_data.quat.y;
    esp_msgs.buff[11].f64 = imu_data.quat.z;
    esp_msgs.buff[12].f64 = imu_data.quat.w;

    encode_msg2buffer(&esp_msgs, tx_msg_buffer, sizeof(tx_msg_buffer));
    Serial.write(tx_msg_buffer, (esp_msgs.buff_size * 8) + 4);
    Serial.println(' ');
  }
}

// Serial.print(Mobile.qd_target[0]);
// Serial.print(" ");
// Serial.print(Mobile.qd_target[1]);
// Serial.print(" ");
// Serial.print(Mobile.qd_target[2]);
// Serial.print(" ");
// Serial.print(Mobile.qd_target[3]);
// Serial.print(" ");

// Serial.print(Mobile.fb_qd[0]);
// Serial.print(" ");
// Serial.print(Mobile.fb_qd[1]);
// Serial.print(" ");
// Serial.print(Mobile.fb_qd[2]);
// Serial.print(" ");
// Serial.print(Mobile.fb_qd[3]);
// Serial.print(" ");

// Serial.print(15);
// Serial.print(" ");
// Serial.println(-15);
