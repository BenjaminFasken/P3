F = : g = 9.82: 
m_1 = : m_2 = : m_3 = : m_4 = : m_5 = : 
T_01 = : T_12 = : T_23 = : T_34 = : T_45 = : 
f_01 = : f_12 = : f_23 = : f_34 = : f_45 = : 
s_1 = : s_2 = : s_3 = : s_4 = : s_5 = : 
s_c1 = : s_c2 = : s_c3 = : s_c4 = : s_c5 = : 

R_1, R_2, R_3, R_4, R_5 = load_rot_matrix(): 

d5 = R_5(T_45 + cross(-s_c5,f_45) + cross(s_5-s_c5,-F))
d4 = R_4(T_34 - T_45 + cross(-s_c4,f_34) + cross(s_4-s_c4,-f_45))
d3 = R_3(T_23 - T_34 + cross(-s_c3,f_23) + cross(s_3-s_c3,-f_34))
d2 = R_2(T_12 - T_23 + cross(-s_c2,f_12) + cross(s_2-s_c2,-f_23))
d1 = R_1(T_01 - T_12 + cross(-s_c1,f_01) + cross(s_1-s_c1,-f_12))

