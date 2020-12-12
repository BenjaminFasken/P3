%% Clears every time file is run
clc; clear;

%% Inverse Dynamics Has Now Begun

% Variables for rotation matrix are defined
syms alpha theta real

% Rotation matrix is defined
ROT(alpha, theta)=[cos(theta)           , -sin(theta)          , 0;
                   sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha);
                   sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha)];
              
% The g- and z-vector are defined
z = [0; 0; 1];

% syms c real

g = [0; 0; -9.82];

% Mass and length of each free body is defined
m = [0.215, 0.2347, 0.1601, 0.0102, 0.1977];
l = [0.061, 0.218, 0.146, 0.030, 0.118];

J1 = [0.0005233, 0, -0.0000003;
      0, 0.0005134, -0.0000059;
      -0.0000003, -0.0000059, 0.0002230];
  
J2 = [0.000055, 0.000012, -0.000018;
      0.000012, 0.007697, 0;
      -0.000018, 0, 0.007676];

J3 = [0.0196654, -0.0034903, -0.0019400;
      -0.0034903, 0.0053028, 0.0080655;
      -0.00194, 0.0080655, 0.0163071];
  
J4 = [0.0025274, -0.0002923, -0.000123;
      -0.0002923, 0.0004300, 0.000906;
      -0.000123, 0.000906, 0.0021798];
  
J5 = [0.00084413, 0.00000008, -0.0002405;
      0.00000008, 0.00094806, -0.00000055;
      -0.0002405, -0.00000055, 0.00013985];
%% Break for not breaking the pc 

% Rotation matrices are defined with respect to global frame
syms theta1 theta2 theta3 theta4 theta5 real
syms d_theta1 d_theta2 d_theta3 d_theta4 d_theta5 real
syms dd_theta1 dd_theta2 dd_theta3 dd_theta4 dd_theta5 real


R1 = ROT(0, theta1);
R2 = R1*ROT(pi/2, theta2);
R3 = R2*ROT(0, theta3);
R4 = R3*ROT(0, theta4);
R5 = R4*ROT(0,pi/2)*ROT(pi/2, theta5);

% The length of each body with respect to global frame is defined
s1 = R1*[0; 0; l(1)];
s2 = R2*[l(2); 0; 0];
s3 = R3*[l(3); 0; 0];
s4 = R4*[l(4); 0; 0];
s5 = R5*[0; 0; l(5)];

% Distance to center of mass is defined from point of rotation
s_c1 = R1*[-0.0000373; -0.0002583; 0.0275726];
s_c2 = R2*[0.181214; -0.000361; 0.000521];
s_c3 = R3*[0.1161588; 0.0004274; 0.0000242];
s_c4 = R4*[0.0201507; 0; -0.0010633];
s_c5 = R5*[-0.01748723; -0.00004897; 0.05195337];

% Distance from global frame to center of mass for each body is defined
r_c1 = s_c1;
r_c2 = s1 + s_c2;
r_c3 = s1 + s2 + s_c3;
r_c4 = s1 + s2 + s3 + s_c4;
r_c5 = s1 + s2 + s3 + s4 + s_c5;

% Angle velocities (w) with respect to global frame
% d_theta values are constructed

w1 = [0; 0; d_theta1];
w2 = w1 + d_theta2 * R2 * z;
w3 = w2 + d_theta3 * R3 * z;
w4 = w3 + d_theta4 * R4 * z;
w5 = w4 + d_theta5 * R5 * z;

% Velocities of each body tip with respect to the global frame
v1 = cross(w1, s1);
v2 = v1 + cross(w2, s2);
v3 = v2 + cross(w3, s3);
v4 = v3 + cross(w4, s4);
v5 = v4 + cross(w4, s5);

% Velocities of each body at CoM with respect to global frame
v_c1m = cross(w1, s_c1);
v_c1=simplify(expand(v_c1m));
v_c2m = v1 + cross(w2, s_c2);
v_c2=simplify(expand(v_c2m));
v_c3m = v2 + cross(w3, s_c3);
v_c3=simplify(expand(v_c3m));
v_c4m = v3 + cross(w4, s_c4);
v_c4=simplify(expand(v_c4m));
v_c5m = v4 + cross(w5, s_c5);
v_c5=simplify(expand(v_c5m));

% Kinetic energy of each body is defined
T1m = 1/2*m(1)*dot(v_c1, v_c1) + 1/2 * dot(w1, ((R1*J1*transpose(R1))*w1));
T1=simplify(expand(T1m))
T2m = 1/2*m(2)*dot(v_c2, v_c2) + 1/2 * dot(w2, (R2*J2*transpose(R2)*w2));
T2=simplify(expand(T2m))
T3m = 1/2*m(3)*dot(v_c3, v_c3) + 1/2 * dot(w3, (R3*J3*transpose(R3)*w3));
T3=simplify(expand(T3m))
T4m = 1/2*m(4)*dot(v_c4, v_c4) + 1/2 * dot(w4, (R4*J4*transpose(R4)*w4));
T4=simplify(expand(T4m))
T5m = 1/2*m(5)*dot(v_c5, v_c5) + 1/2 * dot(w5, (R5*J5*transpose(R5)*w5));
%%
T5mm=simplify(T5m)
%%
T5=expand(T5mm)
%%
% Potential energy of each body is defined
V1 = m(1) * dot(-g, r_c1);
V2 = m(2) * dot(-g, r_c2);
V3 = m(3) * dot(-g, r_c3);
V4 = m(4) * dot(-g, r_c4);
V5 = m(5) * dot(-g, r_c5);

% The Lagrange equation is defined
Lm = T1 - V1 + T2 - V2 + T3 - V3 + T4 - V4 + T5 - V5
%%
Lmm=simplify(Lm)
%%
Lmmm=expand(Lmm)
%%
L=vpa(Lmmm)
%% Break for not breaking the PC
% Partial derivative of kinetic energy
pd_T1 = diff(L, d_theta1);
pd_T2 = diff(L, d_theta2);
pd_T3 = diff(L, d_theta3);
pd_T4 = diff(L, d_theta4);
pd_T5 = diff(L, d_theta5);

% Partial derivative of potential energy
pd_V1 = diff(L, theta1);
pd_V2 = diff(L, theta2);
pd_V3 = diff(L, theta3);
pd_V4 = diff(L, theta4);
pd_V5 = diff(L, theta5);

% General derivative on top of partial derivative on kinetic energy
d_pd_T1 = diff(pd_T1, theta1) * d_theta1 + diff(pd_T1, d_theta1)*dd_theta1 + diff(pd_T1, theta2) * d_theta2 + diff(pd_T1, d_theta2) * dd_theta2 + diff(pd_T1, theta3) * d_theta3 + diff(pd_T1, d_theta3) * dd_theta3 + diff(pd_T1, theta4) * d_theta4 + diff(pd_T1, d_theta4) * dd_theta4 + diff(pd_T1, theta5) * d_theta5 + diff(pd_T1, d_theta5) * dd_theta5;
d_pd_T2 = diff(pd_T2, theta1) * d_theta1 + diff(pd_T2, d_theta1)*dd_theta1 + diff(pd_T2, theta2) * d_theta2 + diff(pd_T2, d_theta2) * dd_theta2 + diff(pd_T2, theta3) * d_theta3 + diff(pd_T2, d_theta3) * dd_theta3 + diff(pd_T2, theta4) * d_theta4 + diff(pd_T2, d_theta4) * dd_theta4 + diff(pd_T2, theta5) * d_theta5 + diff(pd_T2, d_theta5) * dd_theta5;
d_pd_T3 = diff(pd_T3, theta1) * d_theta1 + diff(pd_T3, d_theta1)*dd_theta1 + diff(pd_T3, theta2) * d_theta2 + diff(pd_T3, d_theta2) * dd_theta2 + diff(pd_T3, theta3) * d_theta3 + diff(pd_T3, d_theta3) * dd_theta3 + diff(pd_T3, theta4) * d_theta4 + diff(pd_T3, d_theta4) * dd_theta4 + diff(pd_T3, theta5) * d_theta5 + diff(pd_T3, d_theta5) * dd_theta5;
d_pd_T4 = diff(pd_T4, theta1) * d_theta1 + diff(pd_T4, d_theta1)*dd_theta1 + diff(pd_T4, theta2) * d_theta2 + diff(pd_T4, d_theta2) * dd_theta2 + diff(pd_T4, theta3) * d_theta3 + diff(pd_T4, d_theta3) * dd_theta3 + diff(pd_T4, theta4) * d_theta4 + diff(pd_T4, d_theta4) * dd_theta4 + diff(pd_T4, theta5) * d_theta5 + diff(pd_T4, d_theta5) * dd_theta5;
d_pd_T5 = diff(pd_T5, theta1) * d_theta1 + diff(pd_T5, d_theta1)*dd_theta1 + diff(pd_T5, theta2) * d_theta2 + diff(pd_T5, d_theta2) * dd_theta2 + diff(pd_T5, theta3) * d_theta3 + diff(pd_T5, d_theta3) * dd_theta3 + diff(pd_T5, theta4) * d_theta4 + diff(pd_T5, d_theta4) * dd_theta4 + diff(pd_T5, theta5) * d_theta5 + diff(pd_T5, d_theta5) * dd_theta5;

% Now the 5 torque values will be defined
tau1 = d_pd_T1 - pd_V1;
tau2 = d_pd_T2 - pd_V2;
tau3 = d_pd_T3 - pd_V3;
tau4 = d_pd_T4 - pd_V4;
tau5 = d_pd_T5 - pd_V5;
%%
simpletau1 = simplify(expand(tau1));
moreSimpletau1 = vpa(simpletau1)
%%
simpletau2 = simplify(expand(tau2));
moreSimpletau2 = vpa(simpletau2)
%%
simpletau3 = simplify(expand(tau3));  
moreSimpletau3 = vpa(simpletau3)
%%
simpletau4 = simplify(expand(tau4));  
moreSimpletau4 = vpa(simpletau4)
%%
simpletau5 = simplify(expand(tau5));  
moreSimpletau5 = vpa(simpletau5)
%%

theta = [0.4;
         0.4;
         0.4;
         0.4;
         0.4];
d_theta = [0.4;
           0.4;
           0.4;
           0.4;
           0.4];
dd_theta = [0.4;
            0.4;
            0.4;
            0.4;
            0.4];
%% Break for not breaking the PC
taum1=subs(moreSimpletau1, {theta1,theta2,theta3,theta4,theta5,d_theta1,d_theta2,d_theta3,d_theta4,d_theta5,dd_theta1,dd_theta2,dd_theta3,dd_theta4,dd_theta5}, {theta(1),theta(2),theta(3),theta(4),theta(5),d_theta(1),d_theta(2),d_theta(3),d_theta(4),d_theta(5),dd_theta(1),dd_theta(2),dd_theta(3),dd_theta(4),dd_theta(5)});
t1=vpa(taum1)
%%
taum2=subs(moreSimpletau2, {theta1,theta2,theta3,theta4,theta5,d_theta1,d_theta2,d_theta3,d_theta4,d_theta5,dd_theta1,dd_theta2,dd_theta3,dd_theta4,dd_theta5}, {theta(1),theta(2),theta(3),theta(4),theta(5),d_theta(1),d_theta(2),d_theta(3),d_theta(4),d_theta(5),dd_theta(1),dd_theta(2),dd_theta(3),dd_theta(4),dd_theta(5)});
t2=vpa(taum2)
%%
taum3=subs(moreSimpletau3, {theta1,theta2,theta3,theta4,theta5,d_theta1,d_theta2,d_theta3,d_theta4,d_theta5,dd_theta1,dd_theta2,dd_theta3,dd_theta4,dd_theta5}, {theta(1),theta(2),theta(3),theta(4),theta(5),d_theta(1),d_theta(2),d_theta(3),d_theta(4),d_theta(5),dd_theta(1),dd_theta(2),dd_theta(3),dd_theta(4),dd_theta(5)});
t3=vpa(taum3)
%%
taum4=subs(moreSimpletau4, {theta1,theta2,theta3,theta4,theta5,d_theta1,d_theta2,d_theta3,d_theta4,d_theta5,dd_theta1,dd_theta2,dd_theta3,dd_theta4,dd_theta5}, {theta(1),theta(2),theta(3),theta(4),theta(5),d_theta(1),d_theta(2),d_theta(3),d_theta(4),d_theta(5),dd_theta(1),dd_theta(2),dd_theta(3),dd_theta(4),dd_theta(5)});
t4=vpa(taum4)
%%
taum5=subs(moreSimpletau5, {theta1,theta2,theta3,theta4,theta5,d_theta1,d_theta2,d_theta3,d_theta4,d_theta5,dd_theta1,dd_theta2,dd_theta3,dd_theta4,dd_theta5}, {theta(1),theta(2),theta(3),theta(4),theta(5),d_theta(1),d_theta(2),d_theta(3),d_theta(4),d_theta(5),dd_theta(1),dd_theta(2),dd_theta(3),dd_theta(4),dd_theta(5)});
t5=vpa(taum5)
%%
t=[t1;
   t2;
   t3;
   t4;
   t5]