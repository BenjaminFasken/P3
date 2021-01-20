from dynamics_lib import crc
# import library for serial connection
import serial

# initialize serial connection to the 'COM&' port with a baud rate of 115200.
serial_bot = serial.Serial('COM6', 115200, timeout=1)
# ----------||---header------------||254-||---12-----||-sync read-||from address 128-||-to addr 136-||---from motor 1,2,3,4,5-----||---crc---||
# ----------||---header------------||-ID-||--length--||instruction||---address-------||--8 bytes----||---from motor 1,2,3,4,5-----||---crc---||
sync_read = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0C, 0x00, 0x82,        0x80, 0x00,        0x08, 0x00,     0x01, 0x02, 0x03, 0x04, 0x05, 0xA5, 0x64]

# this is the same as:

sync_read = [ 255,  255,  253,    0,  254,   12,    0,  130,         128,    0,           8,    0,        1,    2,    3,    4,    5,  165,  100]
serial_bot.write(sync_read)
dynamixel_motion_data = serial_bot.read(95)
print(dynamixel_motion_data)

int_data = list(dynamixel_motion_data)
theta = [0.0, 0.0, 0.0, 0.0, 0.0]
vel = [0.0, 0.0, 0.0, 0.0, 0.0]
for i in range(5):
    theta[i] = int.from_bytes(dynamixel_motion_data[i * 19 + 13:i * 19 + 17], 'little', signed=True) * 0.001536
    vel[i] = int.from_bytes(dynamixel_motion_data[i * 19 + 9:i * 19 + 13], 'little', signed=True) * 0.00153398

print(vel, theta)


# decoding and crc of string
data = b'\xff\xff\xfd\x00\x01\x0c\x00U\x00\x00\x00\x00\x00\xd9\x0f\x00\x00,' \
       b'\x9c\xff\xff\xfd\x00\x02\x0c\x00U\x00\x00\x00\x00\x00u\x04\x00\x00\x00l\xff\xff\xfd\x00\x03\x0c\x00U\x00\x00' \
       b'\x00\x00\x00\xc2\x04\x00\x00\xb5?\xff\xff\xfd\x00\x04\x0c\x00U\x00\x00\x00\x00\x00\xfc\x06\x00\x00\x11Y\xff' \
       b'\xff\xfd\x00\x05\x0c\x00U\x00\x00\x00\x00\x00q\x0b\x00\x00I\xc2 '
a = list(data)

for i in range(5):
    stat = a[i * 19:(i + 1) * 19]
    if crc.validate_crc(stat):
        print(234)
