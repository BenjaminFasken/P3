import symengine as se

# Define symbols
theta_1 = se.Symbol('theta[1]')
theta_2 = se.Symbol('theta[2]')
theta_3 = se.Symbol('theta[3]')
theta_4 = se.Symbol('theta[4]')
theta_5 = se.Symbol('theta[5]')

d_theta_1 = se.Symbol('d_theta[1]')
d_theta_2 = se.Symbol('d_theta[2]')
d_theta_3 = se.Symbol('d_theta[3]')
d_theta_4 = se.Symbol('d_theta[4]')
d_theta_5 = se.Symbol('d_theta[5]')

dd_theta_1 = se.Symbol('dd_theta[1]')
dd_theta_2 = se.Symbol('dd_theta[2]')
dd_theta_3 = se.Symbol('dd_theta[3]')
dd_theta_4 = se.Symbol('dd_theta[4]')
dd_theta_5 = se.Symbol('dd_theta[5]')

# Constants
g = se.Matrix([0, 0, -9.81])
z = se.Matrix([0, 0, 1])

# Mass of each free body
m_1 = 0.1
m_2 = 0.2
m_3 = 0.1
m_4 = 0.1 / 2
m_5 = 0.2

# Length of each free body
l_1 = 0.1
l_2 = 0.2
l_3 = 0.1
l_4 = -0.1 / 2
l_5 = 0.1

# Inertia matrices of bodies
J_1 = se.diag(0.5, 0.1, 0.5)
J_2 = se.diag(0.1, 0.2, 0.1)
J_3 = se.diag(0.5, 0.1, 0.1 / 3)
J_4 = se.diag(0.1 / 3, 0.2, 0.1 / 4)
J_5 = se.diag(0.1, 0.2, 0.1)

# ROT function
def ROT(alpha, theta_val):
    return se.Matrix([
        [se.cos(theta_val), -se.sin(theta_val), 0],
        [se.sin(theta_val) * se.cos(alpha), se.cos(theta_val) * se.cos(alpha), -se.sin(alpha)],
        [se.sin(theta_val) * se.sin(alpha), se.cos(theta_val) * se.sin(alpha), se.cos(alpha)]
    ])

# Rotation matrices with respect to global frame
R_1 = ROT(0, theta_1)
R_2 = R_1 * ROT(se.pi / 2, theta_2)
R_3 = R_2 * ROT(0, theta_3)
R_4 = R_3 * ROT(0, theta_4 + se.pi / 2)
R_5 = R_4 * ROT(se.pi / 2, -theta_5)

# Length vectors with respect to global frame
s_1 = R_1 * se.Matrix([0, 0, l_1])
s_2 = R_2 * se.Matrix([l_2, 0, 0])
s_3 = R_3 * se.Matrix([l_3, 0, 0])
s_4 = R_4 * se.Matrix([0, 0, l_4])
s_5 = R_5 * se.Matrix([0, 0, l_5])

# Distance to center of mass
s_c_1 = s_1 * 0.5
s_c_2 = s_2 * 0.5
s_c_3 = s_3 * 0.5
s_c_4 = s_4 * 0.5
s_c_5 = s_5 * 0.5

# Distance from global frame to center of mass of each body
r_c_1 = s_c_1
r_c_2 = s_1 + s_c_2
r_c_3 = s_1 + s_2 + s_c_3
r_c_4 = s_1 + s_2 + s_3 + s_c_4
r_c_5 = s_1 + s_2 + s_3 + s_4 + s_c_5

# Angular velocities with respect to the global frame
omega_1 = se.Matrix([0, 0, d_theta_1])
omega_2 = omega_1 + d_theta_2 * (R_2 * z)
omega_3 = omega_2 + d_theta_3 * (R_3 * z)
omega_4 = omega_3 + d_theta_4 * (R_4 * z)
omega_5 = omega_4 + d_theta_5 * (R_5 * z)

# Velocities of body tips with respect to the global frame
v_1 = omega_1.cross(s_1)
v_2 = v_1 + omega_2.cross(s_2)
v_3 = v_2 + omega_3.cross(s_3)
v_4 = v_3 + omega_4.cross(s_4)
v_5 = v_4 + omega_5.cross(s_5)

# Velocities of each body CoM with respect to global frame
v_c_1 = omega_1.cross(s_c_1)
v_c_2 = v_1 + omega_2.cross(s_c_2)
v_c_3 = v_2 + omega_3.cross(s_c_3)
v_c_4 = v_3 + omega_4.cross(s_c_4)
v_c_5 = v_4 + omega_5.cross(s_c_5)

# Kinetic energies of each body
T_1 = 0.5 * m_1 * v_c_1.dot(v_c_1) + 0.5 * omega_1.dot(R_1 * J_1 * R_1.T * omega_1)
T_2 = 0.5 * m_2 * v_c_2.dot(v_c_2) + 0.5 * omega_2.dot(R_2 * J_2 * R_2.T * omega_2)
T_3 = 0.5 * m_3 * v_c_3.dot(v_c_3) + 0.5 * omega_3.dot(R_3 * J_3 * R_3.T * omega_3)
T_4 = 0.5 * m_4 * v_c_4.dot(v_c_4) + 0.5 * omega_4.dot(R_4 * J_4 * R_4.T * omega_4)
T_5 = 0.5 * m_5 * v_c_5.dot(v_c_5) + 0.5 * omega_5.dot(R_5 * J_5 * R_5.T * omega_5)

# Potential energies of each body
V_1 = (-m_1 * g).dot(r_c_1)
V_2 = (-m_2 * g).dot(r_c_2)
V_3 = (-m_3 * g).dot(r_c_3)
V_4 = (-m_4 * g).dot(r_c_4)
V_5 = (-m_5 * g).dot(r_c_5)

# Lagrangian
L = se.expand(T_1 - V_1 + T_2 - V_2 + T_3 - V_3 + T_4 - V_4 + T_5 - V_5)

# Partial derivatives with respect to generalized velocities
pd_T1 = se.diff(L, d_theta_1)
pd_T2 = se.diff(L, d_theta_2)
pd_T3 = se.diff(L, d_theta_3)
pd_T4 = se.diff(L, d_theta_4)
pd_T5 = se.diff(L, d_theta_5)

# Partial derivatives with respect to generalized coordinates
pd_V1 = se.diff(L, theta_1)
pd_V2 = se.diff(L, theta_2)
pd_V3 = se.diff(L, theta_3)
pd_V4 = se.diff(L, theta_4)
pd_V5 = se.diff(L, theta_5)

# Total derivatives of partial derivatives with respect to generalized velocities
d_pd_T1 = (
    se.diff(pd_T1, theta_1) * d_theta_1 + se.diff(pd_T1, d_theta_1) * dd_theta_1 +
    se.diff(pd_T1, theta_2) * d_theta_2 + se.diff(pd_T1, d_theta_2) * dd_theta_2 +
    se.diff(pd_T1, theta_3) * d_theta_3 + se.diff(pd_T1, d_theta_3) * dd_theta_3 +
    se.diff(pd_T1, theta_4) * d_theta_4 + se.diff(pd_T1, d_theta_4) * dd_theta_4 +
    se.diff(pd_T1, theta_5) * d_theta_5 + se.diff(pd_T1, d_theta_5) * dd_theta_5
)

d_pd_T2 = (
    se.diff(pd_T2, theta_1) * d_theta_1 + se.diff(pd_T2, d_theta_1) * dd_theta_1 +
    se.diff(pd_T2, theta_2) * d_theta_2 + se.diff(pd_T2, d_theta_2) * dd_theta_2 +
    se.diff(pd_T2, theta_3) * d_theta_3 + se.diff(pd_T2, d_theta_3) * dd_theta_3 +
    se.diff(pd_T2, theta_4) * d_theta_4 + se.diff(pd_T2, d_theta_4) * dd_theta_4 +
    se.diff(pd_T2, theta_5) * d_theta_5 + se.diff(pd_T2, d_theta_5) * dd_theta_5
)

d_pd_T3 = (
    se.diff(pd_T3, theta_1) * d_theta_1 + se.diff(pd_T3, d_theta_1) * dd_theta_1 +
    se.diff(pd_T3, theta_2) * d_theta_2 + se.diff(pd_T3, d_theta_2) * dd_theta_2 +
    se.diff(pd_T3, theta_3) * d_theta_3 + se.diff(pd_T3, d_theta_3) * dd_theta_3 +
    se.diff(pd_T3, theta_4) * d_theta_4 + se.diff(pd_T3, d_theta_4) * dd_theta_4 +
    se.diff(pd_T3, theta_5) * d_theta_5 + se.diff(pd_T3, d_theta_5) * dd_theta_5
)

d_pd_T4 = (
    se.diff(pd_T4, theta_1) * d_theta_1 + se.diff(pd_T4, d_theta_1) * dd_theta_1 +
    se.diff(pd_T4, theta_2) * d_theta_2 + se.diff(pd_T4, d_theta_2) * dd_theta_2 +
    se.diff(pd_T4, theta_3) * d_theta_3 + se.diff(pd_T4, d_theta_3) * dd_theta_3 +
    se.diff(pd_T4, theta_4) * d_theta_4 + se.diff(pd_T4, d_theta_4) * dd_theta_4 +
    se.diff(pd_T4, theta_5) * d_theta_5 + se.diff(pd_T4, d_theta_5) * dd_theta_5
)

d_pd_T5 = (
    se.diff(pd_T5, theta_1) * d_theta_1 + se.diff(pd_T5, d_theta_1) * dd_theta_1 +
    se.diff(pd_T5, theta_2) * d_theta_2 + se.diff(pd_T5, d_theta_2) * dd_theta_2 +
    se.diff(pd_T5, theta_3) * d_theta_3 + se.diff(pd_T5, d_theta_3) * dd_theta_3 +
    se.diff(pd_T5, theta_4) * d_theta_4 + se.diff(pd_T5, d_theta_4) * dd_theta_4 +
    se.diff(pd_T5, theta_5) * d_theta_5 + se.diff(pd_T5, d_theta_5) * dd_theta_5
)

# Assign numerical values
c = 0.4
subs = {
    theta_1: c,
    theta_2: c,
    theta_3: c,
    theta_4: c,
    theta_5: c,
    d_theta_1: c,
    d_theta_2: c,
    d_theta_3: c,
    d_theta_4: c,
    d_theta_5: c,
    dd_theta_1: c,
    dd_theta_2: c,
    dd_theta_3: c,
    dd_theta_4: c,
    dd_theta_5: c
}

# Torques
# tau_1 = (d_pd_T1 - pd_V1).subs(subs).evalf()
# tau_2 = (d_pd_T2 - pd_V2).subs(subs).evalf()
# tau_3 = (d_pd_T3 - pd_V3).subs(subs).evalf()
# tau_4 = (d_pd_T4 - pd_V4).subs(subs).evalf()
# tau_5 = (d_pd_T5 - pd_V5).subs(subs).evalf()
tau_1 = (d_pd_T1 - pd_V1).evalf()
tau_2 = (d_pd_T2 - pd_V2).evalf()
tau_3 = (d_pd_T3 - pd_V3).evalf()
tau_4 = (d_pd_T4 - pd_V4).evalf()
tau_5 = (d_pd_T5 - pd_V5).evalf()
# Save tau values to a file
with open('tau_values.txt', 'w') as file:
    file.write(f"tau[1] = {tau_1}\n")
    file.write(f"tau[2] = {tau_2}\n")
    file.write(f"tau[3] = {tau_3}\n")
    file.write(f"tau[4] = {tau_4}\n")
    file.write(f"tau[5] = {tau_5}")

# Output torques
print(f"tau[1] = {tau_1}")
print(f"tau[2] = {tau_2}")
print(f"tau[3] = {tau_3}")
print(f"tau[4] = {tau_4}")
print(f"tau[5] = {tau_5}")
