function y = forwardKinMatLab(u)
/*      David hartenberg params
    *   Joint   A       α       D       θ
    *   1       0       0       d1      θ1
    *   2       0       90      0       θ2
    *   3       d2      0       0       θ3
%   *   4       d3      0       0       θ4
%
alpha = 0;
a = 0;
d = theta_1
01 = TDH(alpha,a,d,theta)