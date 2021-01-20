import numpy
from dynamics_lib import crc

import serial
import time

time.sleep(1)

t = time.time()
serial_bot = serial.Serial('COM6', 115200, timeout=1)
                                                                                                                 # 15
write_dynamixel_data = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x16, 0x00, 0x83, 0x64, 0x00, 0x02, 0x00, 0x01, 0x32, 0x00, 0x02,
                        0x32, 0x00, 0x03, 0x32, 0x00, 0x04, 0x32, 0x00, 0x05, 0x32, 0x00, 0x00, 0x00]

pwm = numpy.array([50, 50, 50, 50, 50])

lim = 400
h = pwm - (pwm / lim).astype(int) * lim
a = (h & 0xFF)
b = (h & 0xFF00) >> 8

send = write_dynamixel_data
send[13] = a[0]
send[14] = b[0]
send[16] = a[1]
send[17] = b[1]
send[19] = a[2]
send[20] = b[2]
send[22] = a[3]
send[23] = b[3]
send[25] = a[4]
send[26] = b[4]
send[-2], send[-1] = crc.calc_crc(send[:-2])
serial_bot.write(send)
print(time.time()-t)
a = bytearray(send)
print(a)
print(list(a))
