from dynamics_lib import crc, reset_torque
from math import pi


def get_all_motion(serial):
    "Dette er READ instruktionen til at læse motion data fra de første 5 dynamixel motorer (den sidste er gripperen)"
    sync_read = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0C, 0x00, 0x82, 0x80, 0x00, 0x08, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
                 0xA5, 0x64]
    serial.write(sync_read)
    dynamixel_motion_data = serial.read(95)
    "Konverterer byte-værdier til int"
    a = list(dynamixel_motion_data)

    "Dette for-loop er kun beregnet til at håntere checksum, og fejl, som opstår ved aflæsning af checksum."
    for i in range(5):
        "Hver motor leverer 19 bytes af data. ex: motor1: stat = a[0:19]     motor2: stat = a[19:38] etc."
        stat = a[i * 19:(i + 1) * 19]
        """Her ses, om checksummen er ordentlig. Hvis ikke, prøver vi at se, om motoren overhovedet har afgivet data, 
        eller om det bare er checksummen, der er noget galt med"""
        try:
            is_valid = crc.validate_crc(stat)
        except:
            "Ingen data fra motor."
            print("\nStack trace: ", a)
            print("\nError. Cannot find motor {}. Please reconnect.".format(i+1))
            exit(42)
            raise
        if not is_valid:
            print("\nNot valid checksum. Returning")
            reset_torque(serial)
            return 0, 0

    "Disse værdier er til for at kompensere for vinklerne på vores dh-paramtetre. "
    theta = [-pi, -1.5707, -pi, -pi, -1.5707]
    vel = [0.0, 0.0, 0.0, 0.0, 0.0]

    """Her læses motion data. f.eks. 
    motor1: 
    theta = -pi + dynamixel_motion_data[13:17]
    vel   = 0   + dynamixel_motion_data[9:13]
    motor2:
    theta = -pi/2 + dynamixel_motion_data[32:36]
    vel   = 0     + dynamixel_motion_data[28:32]
    
    'little' står for at det er i little-endian. husk 1024 skrives her som 2410
    signed står for, at det kan også kan være negativt tal
    """
    for i in range(5):
        theta[i] += int.from_bytes(dynamixel_motion_data[i * 19 + 13:i * 19 + 17], 'little', signed=True) * 0.001536
        vel[i] += int.from_bytes(dynamixel_motion_data[i * 19 + 9:i * 19 + 13], 'little', signed=True) * 0.00153398

    return theta, vel
