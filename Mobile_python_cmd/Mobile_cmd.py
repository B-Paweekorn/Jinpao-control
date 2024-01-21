import serial
import time
import math
from MCU_msgs import mcu_msgs

esp_msgs = mcu_msgs(msg_size=13, header=77, carry_bit=2)
ser_msgs = mcu_msgs(msg_size=3, header=83, carry_bit=2)

ser = serial.Serial(port= "COM10", baudrate= 460800)
if not ser.is_open: ser.open()
while not ser.is_open:
    print("Serial not available!!!")

ser_time = time.time() + 0.01

ser_msgs.assign_msgs(0, 1)
ser_msgs.assign_msgs(1, 0)
ser_msgs.assign_msgs(2, 0)
buff = ser_msgs.encode_msg2buffer()
# ser.write(bytearray(ser_msgs.encode_msg2buffer()))
ser.write(bytearray(buff))
# print(buff)

while True:

    # if (time.time() >= ser_time):
    #     ser_time = ser_time + 0.01
    #     ser_msgs.assign_msgs(0, 1)
    #     ser_msgs.assign_msgs(1, 0)
    #     ser_msgs.assign_msgs(2, 0)
    #     buff = ser_msgs.encode_msg2buffer()
    #     # ser.write(bytearray(ser_msgs.encode_msg2buffer()))
    #     ser.write(bytearray(buff))
    #     # print(buff)

    if ser.in_waiting:
        esp_msgs.decode_buffer2msg(val_byte=ser.read(1))
        if esp_msgs.status:
            odom_vx = esp_msgs.get_msgs(0)
            odom_vy = esp_msgs.get_msgs(1)
            odom_wz = esp_msgs.get_msgs(2)

            gyro_x = esp_msgs.get_msgs(3)
            gyro_y = esp_msgs.get_msgs(4)
            gyro_z = esp_msgs.get_msgs(5)

            accel_x = esp_msgs.get_msgs(6)
            accel_y = esp_msgs.get_msgs(7)
            accel_z = esp_msgs.get_msgs(8)

            quat_x = esp_msgs.get_msgs(9)
            quat_y = esp_msgs.get_msgs(10)
            quat_z = esp_msgs.get_msgs(11)
            quat_w = esp_msgs.get_msgs(12)

            print(odom_vx, odom_vy, odom_wz)