"""
--- How to run the code ---
Simply copy and paste the following code into your cli (next line)
python3 ms3.py
Then load up Coppelia sim and insert 'ms3.csv'
"""

# Imports
import numpy as np
from numpy import linalg as la
import modern_robotics as mr

# Dimensions as described in the wiki
L = 0.47/2 # Length of the chassis
W = 0.3/2 # Width of the chassis
R = 0.0475

# Values needed prior to calculation
Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0], 
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0], 
                  [0, 0, 1, 0, 0, 0]]).astype(float).T
print(Blist)

robot_config = np.array([0,0,0,0,0,0.2,-1.6,0]).astype(float)

Xd_test = np.array([[0,0,1,0.5],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]]).astype(float)

Xd_next = np.array([[0,0,1,0.6],
                    [0,1,0,0],
                    [-1,0,0,0.3],
                    [0,0,0,1]]).astype(float)

X_test = np.array([[0.170, 0, 0.985, 0.387],
                    [0, 1, 0, 0],
                    [-0.985, 0, 0.170, 0.570],
                    [0, 0, 0, 1]]).astype(float)

Kp = np.zeros((6,6))
Ki = np.zeros((6,6))
dt = 0.01

T_test = np.array([[1, 0, 0, 0.1662],
               [0, 1, 0, 0],
               [0, 0, 1, 0.0026],
               [0, 0, 0, 1]]).astype(float)

M_test = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]]).astype(float)

X_inc_test = np.zeros(6)

def Jacobian_Psuedo_Inverse(T, Blist, M, Configuration):
    """
    As the title suggests, this function calcultes the Jacobian Pseudo Inverser

    Inputs - T - Inital Frame
           - Blist - Screw axis for the five joints
           - M - Inital Config of EE
           - Configuration - 12 Array of the Robot Configuration 
    
    Returns - Jacobian Psuedo Inverse
    """ 
    joint_coords = Configuration[3:8]
    Te = mr.FKinBody(M, Blist, joint_coords)
    F = np.array([[0,0,0,0],
                  [0,0,0,0],
                  [-1/(L+W), 1/(L+W), 1/(L+W),-1/(L+W)], # Borrowed from milestone1
                  [1,1,1,1],
                  [-1,1,-1,1],
                  [0,0,0,0]]).astype(float)
    F = (0.047 / 4) * F
    Jb = mr.Adjoint(mr.TransInv(Te) @ mr.TransInv(T)) @ F
    Ja = mr.JacobianBody(Blist, joint_coords)
    # print(np.hstack((Jb, Ja)))
    return np.linalg.pinv(np.hstack((Jb, Ja)))



# Function FeedbackController
def FeedbackControl(X, X_d, X_dn, Kp, Ki, dt, X_inc):
    """
    This function does the feedback control for the KUKA robot

    Input: X - The current actual end-effector config
           X_d - The current end-effector reference config
           X_dn - The end-effector reference config at the next timestep in the ref. traj
           Kp, Ki - PI gain matrices
           dt - Timestep
           X_inc - Incremented Error Integral
    Returns - The commanded end-effector Twist expressed in the ee frame

    """
    Xd_inv = mr.TransInv(X_d)
    Vd = mr.se3ToVec((1/dt)*mr.MatrixLog6(Xd_inv @ X_dn))
    Ad = mr.Adjoint(mr.TransInv(X) @ X_d)
    Ad_Vd = Ad @ Vd # [Ad_{X^-1 X_d}] Vd
    X_err = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X) @ X_d)) # [X_err] = log(X^-1 X_d)
    X_inc = X_inc + X_err*dt
    V = Ad_Vd + Kp @ X_err + Ki @ X_inc
    # print(Vd)
    # print(Ad_Vd)
    # print(V)
    # print(X_err)
    return V, X_err, X_inc

JPI = Jacobian_Psuedo_Inverse(T_test, Blist, M_test, robot_config)
FBC = FeedbackControl(X_test, Xd_test, Xd_next, Kp, Ki, dt, X_inc_test)