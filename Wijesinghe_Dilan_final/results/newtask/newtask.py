"""
--- How to run the code ---
Simply copy and paste the following code into your cli (next line)
python3 base.py
Then load up Coppelia sim and insert 'best.csv'
"""

# Imports
import numpy as np
import modern_robotics as mr
import sys
sys.path.insert(0,'../../code/')
from ms1 import NextState
from ms2 import TrajectoryGenerator
from ms3 import Jacobian_Psuedo_Inverse, FeedbackControl
import matplotlib.pyplot as plt

og_stdout = sys.stdout
with open('newtask.log', 'wt') as f:
    sys.stdout = f
    print("Starting Simulation")

    in_ = np.pi / 2

    def vecToSE3(theta, vec):
        """
        Helper function for writing an SE3 matrix in Numpy
        """
        return np.array([[np.cos(theta), -np.sin(theta), 0,  vec[0]],
                        [np.sin(theta),  np.cos(theta), 0,  vec[1]],
                        [0,                0,           1,  vec[2]],
                        [0,                0,           0,       1]])

    """ Default Input Values from previous milestones"""
    print("Creating inital Transformation Matrices")
    # Inital config of ee in the ref. traj.
    Tse_init = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])

    # cube's init config
    Tsc_init = np.array([[1, 0, 0, 1],
                        [0, 1, 0, 0.5],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])

    # cube's final config
    Tsc_fin = np.array([[0, 1, 0,  2],
                        [-1, 0, 0, -2],
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
                        [-np.sin(in_), 0, np.cos(in_), 0.01],
                        [0,            0,      0,      1]]) # @ roty
    k = 1

    """ Input for the Initial Configuration """
    print("Creating the inital config")
    phi = 0
    x = -1
    y = 1
    th1 = 0
    th2 = np.pi/6
    th3 = -np.pi/2
    th4 = -np.pi/6
    th5 = 0
    wh1 = 0
    wh2 = 0
    wh3 = 0
    wh4 = 0
    gripper_status = 0
    init_config = np.array([phi,x,y,th1,th2,th3,th4,th5,
                            wh1,wh2,wh3,wh4,gripper_status])

    # Kp and Ki values to tweak
    Kp_test = 0.78
    Ki_test = 0.005
    Kp = np.identity(6)*Kp_test
    Ki = np.identity(6)*Ki_test
    dt = 0.01 # delta t

    """ Values needed for the Jacobian Psuedo Inverse """
    print("Creating Values for Jacobian Psuedo Inverse")
    Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]]).astype(float)

    M0e = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0, 1]]).astype(float)

    Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                    [0, -1, 0, -0.5076, 0, 0], 
                    [0, -1, 0, -0.3526, 0, 0],
                    [0, -1, 0, -0.2176, 0, 0], 
                    [0, 0, 1, 0, 0, 0]]).astype(float).T

    X_inc = np.zeros(6)
    X = Tse_init 
    X_err = []
    it_list = []
    real_config = []
    current_config = init_config # Need this to generalize

    print("Generating the inital Trajectory")
    traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_fin, Tce_grasp, Tce_standoff, k)

    """ Main Loop that runs the update to the current config """
    print("Starting Main Loop")
    for i in range(traj.shape[0]-1):
        print(f"Updating real config for the {i}th time")
        real_config.append(current_config)
        joint_coords = current_config[3:8] # Code from ms3
        T0e = mr.FKinBody(M0e, Blist, joint_coords) 
        Tsb = vecToSE3(current_config[0], [current_config[1], current_config[2], 0.0963]) # Create TF
        X = Tsb @ Tb0 @ T0e # Getting Tse
        X_d = np.array([[traj[i][0], traj[i][1], traj[i][2], traj[i][9]], 
                        [traj[i][3], traj[i][4], traj[i][5], traj[i][10]],
                        [traj[i][6], traj[i][7], traj[i][8], traj[i][11]],
                        [0, 0, 0, 1]])  
        X_dn = np.array([[traj[i+1][0], traj[i+1][1], traj[i+1][2], traj[i+1][9]], 
                        [traj[i+1][3], traj[i+1][4], traj[i+1][5], traj[i+1][10]],
                        [traj[i+1][6], traj[i+1][7], traj[i+1][8], traj[i+1][11]],
                        [0, 0, 0, 1]])              
        print(f"Doing FeedbackControl for the {i}th time")
        currV, currX_err, X_inc = FeedbackControl(X, X_d, X_dn, Kp, Ki, dt, X_inc)
        X_err.append(currX_err)
        JInv = Jacobian_Psuedo_Inverse(Tb0, Blist, M0e, current_config)
        Wheel_V = JInv @ currV

        current_config = NextState(current_config, Wheel_V, dt, 20)
        current_config = np.array([*current_config, traj[i][12]])
        it_list.append(i+1)

    print("Main Loop Finished")

    X_err = np.array(X_err)
    # print(real_config)
    err_x =   X_err[:,0]
    err_y =   X_err[:,1]
    err_z =   X_err[:,2]
    err_r =   X_err[:,3]
    err_p =   X_err[:,4]
    err_yaw = X_err[:,5]

    fig, axs = plt.subplots(2,3)
    axs[0,0].plot(err_x)
    axs[0,1].plot(err_y)
    axs[0,2].plot(err_z)
    axs[1,0].plot(err_r)
    axs[1,1].plot(err_p)
    axs[1,2].plot(err_yaw)

    axs[0,0].set_title('X Error')
    axs[0,1].set_title('Y Error')
    axs[0,2].set_title('Z Error')
    axs[1,0].set_title('R Error')
    axs[1,1].set_title('P Error')
    axs[1,2].set_title('Yaw Error')

    axs[0,0].set(ylabel='Error [m]')
    axs[0,1].set(ylabel='Error [m]')
    axs[0,2].set(ylabel='Error [m]')

    axs[1,0].set(ylabel='Error [rad]')
    axs[1,1].set(ylabel='Error [rad]')
    axs[1,2].set(ylabel='Error [rad]')

    for ax in axs.flat:
        ax.set(xlabel='Time [s^2]')

    fig.tight_layout()
    print("Plotting Error Data")
    plt.savefig('ErrorData.png')
    plt.show()
  
    print("Plotting Twist Error over Time")
    np.savetxt("newtask.csv", real_config, delimiter=',')
    np.savetxt("newtask_err.csv", X_err, delimiter=',')
    t = np.multiply(it_list, dt)
    plt.plot(t, X_err)
    plt.legend(['Roll Err', 'Pitch Error', 'Yaw Error', 'X Error', 'Y Error', 'Z Error'])
    plt.title("Twist Err vs Time")
    plt.xlabel("Time")
    plt.ylabel("Twist Err")
    plt.savefig('TwistError.png')
    plt.show()
    
    sys.stdout = og_stdout