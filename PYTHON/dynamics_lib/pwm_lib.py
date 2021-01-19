import time

from dynamics_lib import crc
import dynamics_lib as dyn
import numpy

PWM_LIMIT = 650

# Motor constants
# MX_106 = [76, 171]
# MX_64 = [155, 122]
# MX_28 = [480, 157]


MX_106 = [171, 76]
MX_64 = [122, 131]
MX_28 = [157, 350]


def send_all_PWM(serial, pwm):
    write_dynamixel_data = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x16, 0x00, 0x83, 0x64, 0x00, 0x02, 0x00, 0x01, 0x32, 0x00,
                            0x02, 0x32, 0x00, 0x03, 0x32, 0x00, 0x04, 0x32, 0x00, 0x05, 0x32, 0x00, 0x00, 0x00]

    for i in range(len(pwm)):
        if pwm[i] > PWM_LIMIT:
            pwm[i] = PWM_LIMIT
        elif pwm[i] < -PWM_LIMIT:
            pwm[i] = -PWM_LIMIT

    a = (pwm & 0xFF)
    b = (pwm & 0xFF00) >> 8

    send = write_dynamixel_data
    for i in range(5):
        send[13 + i * 3] = a[i]
        send[14 + i * 3] = b[i]

    send[-2], send[-1] = crc.calc_crc(send[:-2])
    serial.write(send)


def convert_all_pwm(tau, vel):
    return numpy.array([int(convert_pwm(tau[0], vel[0], MX_64)),
                        int(convert_pwm(tau[1], vel[1], MX_106)),
                        int(convert_pwm(tau[2], vel[2], MX_64)),
                        int(convert_pwm(tau[3], vel[3], MX_28)),
                        int(convert_pwm(tau[4], vel[4], MX_28))
                        ])


def convert_pwm(tau, vel, motor_type):
    return tau * motor_type[0] + vel * motor_type[1]


def update_gripper(serial):
    if dyn.goal_gripper != dyn.gripper_state:
        time.sleep(0.02)
        dyn.gripper_state = dyn.goal_gripper
        pack = [0xFF, 0xFF, 0xFD, 0x00, 0x06, 0x09, 0x00, 0x03, 0x74, 0x00, 0xB1, 0x0D, 0x00, 0x00, 0x5A, 0x7D] \
            if dyn.goal_gripper else [0xFF, 0xFF, 0xFD, 0x00, 0x06, 0x09, 0x00, 0x03, 0x74, 0x00, 0x42, 0x0E, 0x00,
                                      0x00, 0x4E, 0x81]
        serial.write(pack)
        time.sleep(0.02)
