from dynamics_lib import matrix_op_2, pwm_lib, motion, reset_torque, reset, kinematics
import dynamics_lib as dyn
import numpy as np

np.set_printoptions(precision=3, suppress=True)
import serial
import time
import os

serial_bot = serial.Serial('COM6', 115200, timeout=.5)
serial_mram = serial.Serial('COM3', 115200, timeout=.5)
reset(serial_bot)

mram_thread = dyn.MRAM_Thread(serial_mram)
mram_thread.start()
while dyn.goal_theta is None:
    continue

goal_theta = np.zeros(5)
goal_vel = np.zeros(5)
goal_acc = np.zeros(5)
pose_1, pose_2, pose_3 = True, False, False

pi = 3.14159565357989
tot, tick_1, tick_4 = 0, -16, -16
total_time = time.time()
while True:
    tick_time = time.time()
    if tot > tick_4 + 5:
        tick_4 = tot
        file = open("kp_kv_vals.txt", "r", encoding="utf-8")
        kp = eval(file.readline())
        kv = eval(file.readline())
        file.close()
        #
        # if pose_1:
        #     goal_theta = np.array([-pi / 2, pi / 2, 0, 0, 0])
        #     pose_1 = False
        #     pose_2 = True
        # elif pose_2:
        #     goal_theta = np.array([0, 3*pi / 4, pi / 4, pi / 4, pi / 2])
        #     pose_2 = False
        #     pose_3 = True
        # elif pose_3:
        #     goal_theta = np.array([-0.6,pi/2,pi/2,pi/2, pi])
        #     pose_3 = False
        #     pose_1 = True
        # # goal_theta = np.array([0, pi / 2, pi / 2, 0, .01]) if goal_theta[4] == 0 else np.array([0, pi / 2, -pi / 2, 0, 0])

    serial_bot.flushInput()
    theta, vel = motion.get_all_motion(serial_bot)
    if theta == 0:
        reset_torque(serial_bot)
        continue
    elif theta == 1:
        break

    M, V, G = matrix_op_2.update_matrices(theta, vel)
    # error_theta = goal_theta - theta
    error_theta = dyn.goal_theta - theta
    error_vel = goal_vel - vel

    not_tau = goal_acc + kp * error_theta + kv * error_vel
    tau = np.dot(M, not_tau) + V + G

    pwm = pwm_lib.convert_all_pwm(tau, vel)
    pwm_lib.send_all_PWM(serial_bot, pwm)
    pwm_lib.update_gripper(serial_bot)

    if tot > tick_1 + .2:
        tick_1 = tot
        os.system("\033[2J\033[H")
        print("theta:                   {}".format(t := np.array(theta)))
        # print("homo_trans:              {}".format(ht := kinematics.fwd_kin(theta)))
        # print("inv_kin:                 {}".format(kinematics.inv_kin(t)))
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

    # file = open("data.csv", "a")
    # file.write("\n{:10.5f}\t{:10.5f}".format(tot, theta[2]))
    # file.close()

    # def run_script(file_name):
    #