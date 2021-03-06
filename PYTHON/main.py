from dynamics_lib import matrix_op_2, pwm_lib, motion, reset
import dynamics_lib as dyn
import numpy as np
import serial
import time
import os
np.set_printoptions(precision=3, suppress=True)

serial_bot = serial.Serial('COM6', 115200, timeout=.5)
serial_mram = serial.Serial('COM3', 115200, timeout=.5)
reset(serial_bot)

mram_thread = dyn.MRAM_Thread(serial_mram)
mram_thread.start()
while dyn.goal_theta is None:
    continue

goal_vel = np.zeros(5)
goal_acc = np.zeros(5)
kp, kv = [150, 150, 350, 2000, 30000], [100, 190, 80, 100, 20]

tot, terminal_update = 0, -15
total_time = time.time()
while True:
    tick_time = time.time()

    serial_bot.flushInput()
    theta, vel = motion.get_all_motion(serial_bot)
    if theta == 0 and vel == 0:
        continue

    M, V, G = matrix_op_2.update_matrices(theta, vel)
    error_theta = dyn.goal_theta - theta
    error_vel = goal_vel - vel

    modified_acc = goal_acc + kp * error_theta + kv * error_vel
    tau = np.dot(M, modified_acc) + V + G

    pwm = pwm_lib.convert_all_pwm(tau, vel)
    pwm_lib.send_all_PWM(serial_bot, pwm)
    pwm_lib.update_gripper(serial_bot)

    if tot > terminal_update + .2:
        terminal_update = tot
        os.system("\033[2J\033[H")
        print("theta:                   {}".format(t := np.array(theta)))
        print("vel:                     {}".format(np.array(vel)))
        print("error_theta:             {}".format(error_theta))
        print("goal_theta:              {}, {}".format(dyn.goal_theta, dyn.goal_gripper))
        print("error_vel:               {}".format(error_vel))
        print("tau:                     {}".format(tau))
        print("pwm:                     {}".format(pwm) + "\n")

    print(
        "\rsending pwm. Tick time: {:10.4f}\tTotal time: {:10.4f}".format(time.time() - tick_time,
                                                                          tot := time.time() - total_time),
        end='')
