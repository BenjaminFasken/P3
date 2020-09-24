function y = fcn(u)
d1 = 0.23500;
d2 = 0.21675;
d3 = 0.10875;
%d4 = 0.03000;

u = [0.1;0.1;d1;0];
x = u(1);
y = u(2);
z = u(3);
pitch = u(4);

r = sqrt(x^2+y^2+(z-d1)^2)
theta1 = atan2(y,x)
a = atan((z-d1)/sqrt(x^2+y^2))
b = acos((d2^2+r^2-d3^2)/(2*d2*r))
theta2 = atan((z-d1)/sqrt(x^2+y^2)) + acos((d2^2+r^2-d3^2)/(2*d2*r))
theta3 = acos((d2^2 + d3^2 - r^2) / (2 * d2 * d3))
theta4 = theta2+theta3-pitch
q = [theta1;theta2;theta3;theta4];
y = q