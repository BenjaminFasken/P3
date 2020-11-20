function func_bitch = inv_kin(u)
d_1 = 0.061;   d_2 = 0.218;   d_3 = 0.146;   d_4 = 0.03; d_5 = 0.118;

dh45 = TDH(0,pi/2, d_4, pi/2);
dh56 = TDH(0, 0, d_5, 0);
dh67 = TDH(0, pi/2, 0, pi/2);
Target0W = eulerZYX2T(u(1),u(2),u(3),u(4),u(5),u(6));
Target05 = round(Target0W*inv(dh45*dh56*dh67), 7);

x = Target05(1,4);
y = Target05(2,4);
z = Target05(3,4);



r = sqrt(x^2 + y^2 + (z-d_1)^2);
theta1 = atan2(y,x);
if abs(((d_2^2 + r^2 - d_3^2)/(2*d_2*r)-1)^2) < 0.000001
    beta = 0;
else
    beta = acos((d_2^2 + r^2 - d_3^2)/(2*d_2*r));
end
theta2 = atan2((z-d_1),sqrt(x^2+y^2)) + beta;

if abs(((d_2^2 + d_3^2 - r^2) / (2 * d_2 * d_3)+1)^2) < 0.000001
    gamma = 0;
else
    gamma = acos((d_2^2 + d_3^2 - r^2) / (2 * d_2 * d_3))-pi;
end
theta3 = gamma;

dh01 = round(TDH(0, 0, d_1, theta1), 7);
dh12 = round(TDH(0, pi/2, 0, theta2),7);
dh23 = TDH(d_2, 0, 0, theta3);
T03 = dh01*dh12*dh23;

T35 = inv(T03)*Target0W 
T2eulerXYZ(T35);

theta4 = atan2(T35(2,1),T35(1,1));
theta5 = atan2(T35(3,3),-T35(3,2));
q = [theta1; theta2; theta3; theta4; theta5];
func_bitch = q;