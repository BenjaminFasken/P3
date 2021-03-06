from dynamics_lib import crc, reset_torque
from math import pi


def get_all_motion(serial):
    sync_read = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0C, 0x00, 0x82, 0x80, 0x00, 0x08, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
                 0xA5, 0x64]
    serial.write(sync_read)
    dynamixel_motion_data = serial.read(95)

    a = list(dynamixel_motion_data)
    for i in range(5):
        stat = a[i * 19:(i + 1) * 19]
        try:
            is_valid = crc.validate_crc(stat)
        except:
            print("\nStack trace: ", a)
            print("\nError. Cannot find motor {}. Please reconnect.".format(i+1))
            exit(42)
            raise
        if not is_valid:
            print("\nNot valid checksum. Returning")
            reset_torque(serial)
            return 0, 0
        
    theta = [-pi, -1.5707, -pi, -pi, -1.5707]
    vel = [0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(5):
        theta[i] += int.from_bytes(dynamixel_motion_data[i * 19 + 13:i * 19 + 17], 'little', signed=True) * 0.001536
        vel[i] += int.from_bytes(dynamixel_motion_data[i * 19 + 9:i * 19 + 13], 'little', signed=True) * 0.00153398

    return theta, vel
