from dynamics_lib import matrix_op_2, pwm_lib, motion, reset
import dynamics_lib as dyn
import numpy as np
import serial
import time
import os
"Gør sådan, at når vi skal printe til terminalen, så har tallene kun 3 decimaler"
np.set_printoptions(precision=3, suppress=True)

"Skab seriel forbindelse til både robotten og MRAM"
serial_bot = serial.Serial('COM6', 115200, timeout=.5)
serial_mram = serial.Serial('COM3', 115200, timeout=.5)
"Reset gør, at robotten slukkes, tændes, og derefter sætter holding torque til true"
reset(serial_bot)

"Skab en ny tråd på CPU'en, som skal håntere input fra MRAM"
mram_thread = dyn.MRAM_Thread(serial_mram)
mram_thread.start()
"Vent indtil goal_theta initieres fra MRAM. "
while dyn.goal_theta is None:
    continue

"Her initieres goal_vel, goal_acc, kp, og kv værdierne"
goal_vel = np.zeros(5)
goal_acc = np.zeros(5)
kp, kv = [150, 150, 350, 2000, 30000], [100, 190, 80, 100, 20]

"""Her initieres tot; den totale tid, programmet har kørt.
og terminal_update; Den tid, hvor terminalen sidst blev opdateret"""
tot, terminal_update = 0, -15
total_time = time.time()

"""Et uendeligt loop startes. Dette er hvor vi:
1: indtager data fra MRAM og Robotten
2: udregner torque
3: sender PWM til Robotten"""
while True:
    "Dette er til at udregne tiden for ét loop"
    tick_time = time.time()

    "Fjern uønsket data. Dette kunne være error messages etc."
    serial_bot.flushInput()
    "Få data fra robotten"
    theta, vel = motion.get_all_motion(serial_bot)
    "motion.get_all_motion  returnerer 0, hvis vi ikke kunne få noget ordentligt data"
    if theta == 0 and vel == 0:
        "continue betyder, at vi skipper resten af dette loop, og begynder forfra på det næste loop"
        continue

    "Her opdateres Matricerne mht. Robottens motion data"
    M, V, G = matrix_op_2.update_matrices(theta, vel)
    "udregner fejlene (1)"
    error_theta = dyn.goal_theta - theta
    error_vel = goal_vel - vel

    "Summerer (1) med goal_acc for at få Tau^mærke "
    modified_acc = goal_acc + kp * error_theta + kv * error_vel
    "Udregner torque"
    tau = np.dot(M, modified_acc) + V + G

    "converterer torque til PWM og sender PWM"
    pwm = pwm_lib.convert_all_pwm(tau, vel)
    pwm_lib.send_all_PWM(serial_bot, pwm)
    pwm_lib.update_gripper(serial_bot)

    "Alt hernedefter opdaterer terminalen"
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
