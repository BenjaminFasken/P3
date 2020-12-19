function y = fwd_kin(u)
d_1 = 0.061;   d_2 = 0.218;   d_3 = 0.146;   d_4 = 0.03; d_5 = 0.118;
t_1 = u(1); t_2 = u(2); t_3 = u(3); t_4 = u(4); t_5 = u(5); 
%     David hartenberg params
 %   *   Joint   A       α       D       θ
  %  *   1       0       0       d1      θ1
   % *   2       0       90      0       θ2
    %*   3       d2      0       0       θ3
   % *   4       d3      0       0       θ4+90
  %  *   5       0       90      d4      0
% t_1 = pi/4; t_2 = pi/4; t_3 = pi/4; t_4 = pi/4;
% t_1 = 0; t_2 = 0; t_3 = 0; t_4 = 0;

dh01 = TDH(0, 0, d_1, t_1);
dh12 = TDH(0, pi/2, 0, t_2);
dh23 = TDH(d_2, 0, 0, t_3);
dh34 = TDH(d_3, 0, 0, t_4 + pi/2);
dh45 = TDH(0, pi/2, d_4, t_5 + pi/2);
dh56 = TDH(0, 0, d_5, 0);
dh67 = TDH(0, pi/2, 0, pi/2);

a = dh01*dh12*dh23*dh34*dh45*dh56*dh67;
a = round(a,7)
y = T2eulerXYZ(a);