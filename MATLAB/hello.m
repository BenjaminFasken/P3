clear; clc;

w = [pi/4, pi/4, -pi/4, pi/4]

q = fwd_kin(w)
u = inv_kin(q)
fwd_kin(u)