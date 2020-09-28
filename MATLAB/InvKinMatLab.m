function func_bitch = fcn(u)
d1 = 0.23500;   d2 = 0.21675;   d3 = 0.10875;   d4 = 0.03000;

u = [d2+d3+d4-1e-15;0;d1;0*pi/2];

theta1 = atan2(u(2),u(1));
pitch = -u(4)+pi/2;
yaw = theta1;
T4W=[ 1 0 0 0;        0 1 0 0;         0 0 1 d4;    0 0 0 1];
 
Target0W = eulerZYX2T(u(1),u(2),u(3),yaw,pitch,0)
Target04 = Target0W*inv(T4W)

x = Target04(1,4);
y = Target04(2,4);
z = Target04(3,4);


r = sqrt(x^2 + y^2 + (z-d1)^2);
theta2 = atan((z-d1)/sqrt(x^2+y^2)) + acos((d2^2 + r^2 - d3^2)/(2*d2*r));
theta3 = acos((d2^2 + d3^2 - r^2) / (2 * d2 * d3))-2*pi/2;
theta4 = theta2 + theta3 - pitch+pi/2;
q = [theta1; theta2; theta3; theta4];
func_bitch = q;