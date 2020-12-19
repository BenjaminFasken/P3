import time
import numpy
import threading

from dynamics_lib import kinematics
from dynamics_lib.kinematics import inv_kin

cond = threading.Condition()
goal_theta = None
goal_gripper = 0
gripper_state = 0


def reset(serial_bot):
    serial_bot.flush()
    a = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x03, 0x00, 0x08, 0x07, 0x42]
    b = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x13, 0x00, 0x83, 0x40, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x01, 0x03, 0x01,
         0x04, 0x01, 0x05, 0x01, 0x06, 0x01, 0x0C, 0xEF]
    serial_bot.write(a)
    time.sleep(0.1)
    serial_bot.write(b)


def reset_torque(serial_bot):
    serial_bot.flush()
    a = [255, 255, 253, 0, 254, 6, 0, 3, 64, 0, 0, 46, 22]
    b = [255, 255, 253, 0, 254, 6, 0, 3, 64, 0, 1, 43, 150]
    serial_bot.write(a)
    time.sleep(0.001)
    serial_bot.write(b)


class MRAM_Thread(threading.Thread):
    def __init__(self, serial):
        threading.Thread.__init__(self)
        self.serial = serial

    def run(self):
        global cond, goal_theta, goal_gripper
        while True:
            self.serial.flushInput()
            if self.serial.inWaiting() > 0:
                self.serial.readline()
                a = str(self.serial.readline())
                try:
                    cond.acquire()
                    st_array = a.split(' ')
                    b = numpy.array([float(i) for i in st_array[1:6]]) * 0.001536
                    goal_gripper = int(st_array[6])
                    if len(b) != 5 :
                        continue
                    cart_pos = kinematics.check_pos(b)
                    if cart_pos[2] < 0.03:
                        new_goal = [cart_pos[0], cart_pos[1], 0.03, cart_pos[3], cart_pos[4], cart_pos[5]]
                        new_theta = inv_kin(new_goal, goal_theta)
                        goal_theta = b
                        goal_theta[1] = new_theta[1]
                        goal_theta[2] = new_theta[2]
                    else:
                        goal_theta = b
                    cond.notify_all()
                    cond.release()
                    # print("\r {}".format(goal_theta))
                    time.sleep(0.001)
                except:
                    continue
