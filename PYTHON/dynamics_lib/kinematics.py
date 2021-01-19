import numpy as np
from math import sin, cos, pi, atan2, sqrt, acos

pi_2 = pi / 2
"Vores længder for hvert led"
d_1 = 0.24
d_2 = 0.22
d_3 = 0.145
d_4 = 0.15

"DH algo"
def dh(a, alpha, d, theta):
    return np.array([[cos(theta), -sin(theta), 0, a],
                     [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                     [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d],
                     [0, 0, 0, 1]])

"udregner forward kin mht. vores dh parametre. Vi tager ikke theta5 med, " \
"fordi den ikke gør noget ved end effektorens xyz position"
def fwd_kin(theta):
    h_1 = dh(0, 0, d_1, theta[0])
    h_2 = dh(0, pi_2, 0, theta[1])
    h_3 = dh(d_2, 0, 0, theta[2])
    h_4 = dh(d_3, 0, 0, theta[3])
    h_5 = dh(d_4, 0, 0, 0)
    return h_1.dot(h_2).dot(h_3).dot(h_4).dot(h_5)

"Udregner x,y,z,r,p,y ud fra theta værdier"
def check_pos(theta):
    h = fwd_kin(theta)
    return cart_pos(h)

"""invers kinematik, hvor cart_pos(x,y,z,r,p,y) selvfølgelig er parametre, men også reference-theta værdier,
så vi udregner løsningen, som er tættest på referenceværdierne"""
def inv_kin(cart_pos, theta):
    target_0W = eulerZYX2T(cart_pos)
    T = target_0W.dot(dh(-d_4, 0, 0, 0))
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    "Formelerne står i rapporten, men fremgangsmåden er, at vi udregner theta, udregner de andre løsninger," \
    "hvor derefter vi tager den løsning med mindst afstand fra reference-theta"
    # Theta 1
    theta1 = atan2(y, x)
    t1 = np.array([theta1, theta1-pi, theta1+pi])
    theta1 = t1.flat[np.abs(t1 - theta[0]).argmin()]

    #Theta 2
    r = sqrt(x ** 2 + y ** 2 + (d_1 - z) ** 2)
    if (d_2 ** 2 + r ** 2 - d_3 ** 2) / (2 * d_2 * r) > 1:
        beta = 0
    elif (d_2 ** 2 + r ** 2 - d_3 ** 2) / (2 * d_2 * r) < -1:
        beta = pi
    else:
        beta = acos((d_2 ** 2 + r ** 2 - d_3 ** 2) / (2 * d_2 * r))

    theta2 = atan2((z - d_1), sqrt(x ** 2 + y ** 2))
    t2 = np.array([theta2 + beta, theta2 - beta, pi - theta2 + beta, pi - theta2 - beta])
    theta2 = t2.flat[np.abs(t2 - theta[1]).argmin()]

    # Theta 3
    if (d_2 ** 2 + d_3 ** 2 - r ** 2) / (2 * d_2 * d_3) > 1:
        gamma = 0
    elif (d_2 ** 2 + d_3 ** 2 - r ** 2) / (2 * d_2 * d_3) < -1:
        gamma = pi
    else:
        gamma = acos((d_2 ** 2 + d_3 ** 2 - r ** 2) / (2 * d_2 * d_3)) - pi
    theta3 = gamma
    if abs(theta3 - theta[2]) > abs(theta3 + theta[2]):
        theta3 = -theta3

    # Theta 4
    h_1 = dh(0, 0, d_1, theta1)
    h_2 = dh(0, pi_2, 0, theta2)
    h_3 = dh(d_2, 0, 0, theta3)
    t03 = h_1.dot(h_2).dot(h_3)

    t35 = (np.linalg.inv(t03)).dot(target_0W)

    theta4 = atan2(t35[1, 0], t35[0, 0])
    t4 = np.array([theta4, -theta4])
    theta4 = t4.flat[np.abs(t4 - theta[3]).argmin()]

    return np.array([theta1, theta2, theta3, theta4, theta[4]])


def eulerZYX2T(pos):
    X, Y, Z, rotX, rotY, rotZ = pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]
    return np.array([[cos(rotZ) * cos(rotY), cos(rotZ) * sin(rotY) * sin(rotX) - sin(rotZ) * cos(rotX),
                      cos(rotZ) * sin(rotY) * cos(rotX) + sin(rotZ) * sin(rotX), X],
                     [sin(rotZ) * cos(rotY), sin(rotZ) * sin(rotY) * sin(rotX) + cos(rotZ) * cos(rotX),
                      sin(rotZ) * sin(rotY) * cos(rotX) - cos(rotZ) * sin(rotX), Y],
                     [-sin(rotY), cos(rotY) * sin(rotX), cos(rotY) * cos(rotX), Z],
                     [0, 0, 0, 1]])


"udregning af x,y,z,r,p,y ud fra homogeneous transform"
def cart_pos(T):
    X = T[0, 3]
    Y = T[1, 3]
    Z = T[2, 3]
    p = atan2(-T[2, 0], sqrt(T[0, 0] ** 2 + T[1, 0] ** 2))
    if abs(abs(p) - 90) < 0.000001:
        y = 0
        r = p / abs(p) * atan2(T[0, 1], T[1, 1])
    else:
        y = atan2(T[1, 0] / cos(p), T[0, 0] / cos(p))
        r = atan2(T[2, 1] / cos(p), T[2, 2] / cos(p))

    return np.array([X, Y, Z, r, p, y])
