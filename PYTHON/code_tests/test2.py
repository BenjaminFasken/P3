import time

import numpy
import serial
from dynamics_lib import crc, kinematics
import dynamics_lib as dyn
from dynamics_lib.kinematics import inv_kin
numpy.set_printoptions(precision=3, suppress=True)

serial = serial.Serial('COM3', 115200, timeout=1)

while True:
    serial.flushInput()
    if serial.inWaiting() > 0:
        serial.readline()
        a = str(serial.readline())
        try:
            st_array = a.split(' ')
            goal_theta = numpy.array([float(i) for i in st_array[1:6]]) * 0.001536
            kin = inv_kin(goal_theta,)
            if len(goal_theta) != 5:
                continue
            time.sleep(0.001)
        except:
            continue
