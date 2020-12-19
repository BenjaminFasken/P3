clear; clc;

% w = [pi/4, pi/4, -pi/4, pi/4, pi/4]
%
%  w = [0.0001,pi/4,0.0001,0.0001,0.0001]
%  w = [0.00001, pi/4, 0.000001, pi/2, 0.000001]
 w = [pi/8, pi/2, pi/4, pi/2, 0.000001]
%  w = [0.00001, pi/4, 0.000001, pi/2, 0.000001]

q = round(fwd_kin(w),7)
u = inv_kin(q)
fwd_kin(u)