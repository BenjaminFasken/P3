m_1 = 0.2372; m_2 = 0.2347; m_3 = 0.156; m_4 = 0.0103; m_5 = 0.6969696969; m_F = 0.069696966969; 
z = [0, 0, 1];
T_01 = 0.5*z; T_12 = 0.5*z; T_23 = 0.5*z; T_34 = 0.5*z; T_45 = 0.5*z; 
s_1 = [0, 0, 0.0538]; s_2 = [0.2198, 0, 0]; s_3 = [0.1469, 0, 0]; s_4 = [0.03000, 0, 0]; s_5 = [0, 0, 0.0783]; 
s_c1 = s_1/2; s_c2 = s_2/2; s_c3 = s_3/2; s_c4 = s_4/2; s_c5 = s_5/2; 
g = [0, 0, 9.82]; F_grip = m_F*g; 
acc_1 = [0, 0, 0]; acc_2 = [0, 0, 0]; acc_3 = [0, 0, 0]; acc_4 = [0, 0, 0]; acc_5 = [0, 0, 0]; 
theta = [0,0,0,0,0];
[R_1, R_2, R_3, R_4, R_5] = load_rot_matrix(theta);

f_45 = m_5*acc_5 + F_grip - m_5*g
f_34 = m_4*acc_4 + f_45 - m_4*g
f_23 = m_3*acc_3 + f_34 - m_3*g
f_12 = m_2*acc_2 + f_23 - m_2*g
f_01 = m_1*acc_1 + f_12 - m_1*g

d5 = R_5*transpose(T_45        + cross(-s_c5, f_45) + cross(s_5-s_c5, -F_grip))
d4 = R_4*transpose(T_34 - T_45 + cross(-s_c4, f_34) + cross(s_4-s_c4, -f_45))
d3 = R_3*transpose(T_23 - T_34 + cross(-s_c3, f_23) + cross(s_3-s_c3, -f_34))
d2 = R_2*transpose(T_12 - T_23 + cross(-s_c2, f_12) + cross(s_2-s_c2, -f_23))
d1 = R_1*transpose(T_01 - T_12 + cross(-s_c1, f_01) + cross(s_1-s_c1, -f_12))

w_1 = [0 , 0 , theta(1)];
w_2 = w1 + theta_2*R_2*z
w_3 = w2 + theta_3*R_3*z
w_4 = w3 + theta_4*R_4*z
w_5 = w4 + theta_5*R_5*z

v1 = [0, 0, 0];
v2 = v1 + cross(w2, R_2*s_c2)
v3 = v2 + cross(w3, R_3*s_c3)
v4 = v3 + cross(w4, R_4*s_c4)
v5 = v4 + cross(w5, R_5*s_c5)

v_c1 = [0, 0, 0];
v_c2 = v1 + cross(w2, R_2*s_c2)
v_c3 = v2 + cross(w3, R_3*s_c3)
v_c4 = v3 + cross(w4, R_4*s_c4)
v_c5 = v4 + cross(w5, R_5*s_c5)

T_1 = 
T_2 = 
T_3 = 
T_4 = 
T_5 = 


