function func_bitch = inv_kin(u)
d1 = 0.23500;   d2 = 0.21675;   d3 = 0.10875;   d4 = 0.03000;
%x,y,z,r,p,y
%u = [0.0934    0.0934    0.5182         0    0.7854   -2.3562];
roll = u(4);
pitch = u(5);
yaw = u(6);
dh45 = TDH(d4,-pi/2,0,0);
Target0W = eulerZYX2T(u(1),u(2),u(3),u(4),u(5),u(6))
inv(dh45)
Target04 = round(Target0W*inv(dh45), 7)
%Target04 = Target0W

x = Target04(1,4);
y = Target04(2,4);
z = Target04(3,4);


r = sqrt(x^2 + y^2 + (z-d1)^2);
theta2 = atan2((z-d1),sqrt(x^2+y^2)) + acos((d2^2 + r^2 - d3^2)/(2*d2*r));
r
(d2^2 + r^2 - d3^2)
(2*d2*r)
theta3 = acos((d2^2 + d3^2 - r^2) / (2 * d2 * d3))-2*pi/2;
theta4 = theta2 + theta3 - pitch;
q = [yaw; theta2; theta3; theta4];
func_bitch = q;