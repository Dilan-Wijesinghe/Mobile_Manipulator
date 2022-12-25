"""
--- How to run the code ---
Simply copy and paste the following code into your cli (next line)
python3 ms2.py
Then load up Coppelia sim and insert 'ms2.csv'
"""

# Imports
import numpy as np
from numpy import linalg as la
import modern_robotics as mr


DT = 0.01
in_ = 3*np.pi / 4

"""
TrajectoryGenerator

Input: Tse_init, Tsc_init, Tsc_fin, Tce_stand, Tce_grasp, k
Output: A .csv with the movement trajectories of the end effector
"""
def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_fin, Tce_grasp, Tce_stand, k):
    Tse_stand = Tsc_init @ Tce_stand
    Tse_grasp = Tsc_init @ Tce_grasp

    Tse_stand_final = Tsc_fin @ Tce_stand
    Tse_grasp_final = Tsc_fin @ Tce_grasp

    gripper_close = 0
    for operations in range(0,8):
        if operations == 0:
            N = 4*k/DT # This is for a NUM_SECS time period
            aux = mr.ScrewTrajectory(Tse_init, Tse_stand, 3, N, 5)
            gripper = np.zeros(len(aux)) 

        elif operations == 1:
            N = 2*k/DT # This is for a NUM_SECS time period
            aux_ = mr.CartesianTrajectory(Tse_stand, Tse_grasp, 1, N, 5)
            aux = np.vstack([aux, aux_])
            gripper_open = np.zeros(len(aux_))
            gripper = np.hstack([gripper, gripper_open])

        elif operations == 2:
            aux_ = mr.CartesianTrajectory(Tse_grasp, Tse_grasp, 1, N, 5)
            aux = np.vstack([aux, aux_])
            gripper_close = np.ones(len(aux_))
            gripper = np.hstack([gripper, gripper_close])

        elif operations == 3:
            aux_ = mr.CartesianTrajectory(Tse_grasp, Tse_stand, 1, N, 5)
            aux = np.vstack([aux, aux_])
            gripper_close = np.ones(len(aux_))
            gripper = np.hstack([gripper, gripper_close])

        elif operations == 4:
            N = 4*k/DT # This is for a NUM_SECS time period
            aux_ = mr.ScrewTrajectory(Tse_stand, Tse_stand_final, 3, N, 5)
            aux = np.vstack([aux, aux_])
            gripper_close = np.ones(len(aux_))
            gripper = np.hstack([gripper, gripper_close])

        elif operations == 5:
            N = 2*k/DT # This is for a NUM_SECS time period
            aux_ = mr.CartesianTrajectory(Tse_stand_final, Tse_grasp_final, 1, N, 5)
            aux = np.vstack([aux, aux_])
            gripper_close = np.ones(len(aux_))
            gripper = np.hstack([gripper, gripper_close])
        
        elif operations == 6:
            aux_ = mr.CartesianTrajectory(Tse_grasp_final, Tse_grasp_final, 1, N, 5)
            aux = np.vstack([aux, aux_])
            gripper_open = np.zeros(len(aux_))
            gripper = np.hstack([gripper, gripper_open])
        
        elif operations == 7:
            aux_ = mr.CartesianTrajectory(Tse_grasp_final, Tse_stand_final, 1, N, 5)
            aux = np.vstack([aux, aux_])
            gripper_open = np.zeros(len(aux_))
            gripper = np.hstack([gripper, gripper_open])
        # print(len(aux))
        # print(aux)
    traj = np.zeros((len(aux),13))

    for i in range(len(traj)):
        traj[i][0] = aux[i][0][0]; traj[i][1]  = aux[i][0][1]; traj[i][2]  = aux[i][0][2];
        traj[i][3] = aux[i][1][0]; traj[i][4]  = aux[i][1][1]; traj[i][5]  = aux[i][1][2]
        traj[i][6] = aux[i][2][0]; traj[i][7]  = aux[i][2][1]; traj[i][8]  = aux[i][2][2];
        traj[i][9] = aux[i][0][3]; traj[i][10] = aux[i][1][3]; traj[i][11] = aux[i][2][3];
        traj[i][12] = gripper[i]

    traj_return = traj
          
    # Returns rep of the N configs of the end-effector along the entire concatenated 8-segment 
    # reference trajectory. 
    # Each N reference points are a TF, Tse(i)
    # Csv with the entire 8-seg reference trajectory
    # r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, grasper_state
    # print(out.shape)
    return traj_return

# Inital config of ee in the ref. traj.
Tse_init = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                    [-1, 0, 0, 0.5],
                     [0, 0, 0, 1]])

# cube's init config
Tsc_init = np.array([[1, 0, 0, 1],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])

# cube's final config
Tsc_fin = np.array([[0, 1, 0,  0],
                   [-1, 0, 0, -1],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])

# ee config relative to cube when it is grasping the cube
Tce_standoff = np.array([[np.cos(in_), 0, np.sin(in_),  0],
                         [0,           1,   0,          0],
                         [-np.sin(in_), 0, np.cos(in_), 0.3],
                         [0,            0,      0,      1]])

# ee config hovering above cube pre / post grasp
Tce_grasp = np.array([[np.cos(in_), 0, np.sin(in_),  0],
                      [0,           1,   0,          0],
                      [-np.sin(in_), 0, np.cos(in_), 0.025],
                      [0,            0,      0,      1]]) 

""" Uncomment this section if you want to generate ms2.csv"""
# traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_fin, Tce_grasp, Tce_standoff, 1)
# print(traj)
# np.savetxt('ms2.csv', traj, delimiter=',')