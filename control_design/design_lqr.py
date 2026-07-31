import numpy as np
from scipy.linalg import solve_continuous_are

g = 9.81

# Body + 4 props, from model.sdf / crazyflie_body.xacro (they agree).
# Ignoring Crazyflie.proto's physics of mass = 0.05
m = 0.0282

# Diagonal of the measured inertia tensor
# The real tensor also has small off-diagonal terms (Ixy, Ixz, Iyz), deliberately dropped here
Ixx, Iyy, Izz = 16.5717e-6, 16.6556e-6, 29.2617e-6

A = np.zeros((12, 12))
B = np.zeros((12, 4))

# state: [x',y',z',vx,vy,vz,phi,theta,psi,p,q,r] -> indices 0..11
A[0,3] = 1; A[1,4] = 1; A[2,5] = 1
A[3,7] = g; A[4,6] = -g
A[6,9] = 1; A[7,10] = 1; A[8,11] = 1

# input: [F, taux, tauy, tauz]
B[5,0] = 1/m
B[9,1] = 1/Ixx; B[10,2] = 1/Iyy; B[11,3] = 1/Izz


Q = np.diag([1/0.1**2]*3 + [1/0.5**2]*3 +
            [1/0.2**2, 1/0.2**2, 1/0.5**2] + [1/1.0**2]*3)
R = np.diag([1/(0.5*m*g)**2] + [1/0.01**2]*3)

P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

# check before trusting K in sim, needs negative 
eigs = np.linalg.eigvals(A - B @ K)
print(eigs)
assert np.all(eigs.real < 0), "unstable closed loop, check Q/R or A/B before flying"