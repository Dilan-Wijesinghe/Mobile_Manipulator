"""
--- How to run the code ---
Simply copy and paste the following code into your cli (next line)
python3 ms1.py
Then load up Coppelia sim and insert 'ms1.csv'
"""

# Imports
import numpy as np
import modern_robotics as mr

l = 0.47/2
w = 0.3/2

def cut_off(wheel_speeds, max_speed):
    """
    Function to change the wheel speeds depending on the max_speed
    """
    for i in range(len(wheel_speeds)):       
        wheel_speeds[i] =  max_speed if wheel_speeds[i] > max_speed else wheel_speeds[i]
        wheel_speeds[i] = -max_speed if wheel_speeds[i] < -max_speed else wheel_speeds[i]
    return wheel_speeds

def NextState(curr_config, wheel_speeds, dt, max_ang_speed):
    """ 
    Input: curr_config - 12 Vec of Curr Config (3 chass, 5 arm, 4 wheel ang)
           wheel_speeds - 9 vec of control (4 u, 5 arm joints)
    Returns: A 12-Vector of the config of the robot time dt later
    """
    # --- Values from Config ---
    #  [0:3] - 0,1,2 -> Chassis Config
    #  [3:8] - 3,4,5,6,7 -> Arm Config
    # [8:-1] - 8,9,10,11 -> Wheel Angles 

    # --- Values from Wheel Speeds ---
    # [0:4] - 0,1,2,3 -> Wheel Speeds
    # [4:-1] - 4,5,6,7,8 -> Arm Speeds
    F = np.array([[-1/(l+w), 1/(l+w), 1/(l+w),-1/(l+w)],
                  [1,1,1,1],
                  [-1,1,-1,1]
                ])
    F = (0.047 / 4) * F
    chassis_cfg = curr_config[0:3]
    arm_cfg = curr_config[3:8]
    wheel_cfg = curr_config[8:12]

    wheel_ctrl = wheel_speeds[0:4]
    arm_ctrl = wheel_speeds[4:9]

    wheel_speeds = cut_off(wheel_speeds, max_ang_speed)
    
    new_arm = arm_cfg + arm_ctrl * dt
    new_wheel = wheel_cfg + wheel_ctrl * dt

    V_b = F @ wheel_ctrl
    w_bz = V_b[0]
    v_bx = V_b[1]
    v_by = V_b[2]

    d_qb = None

    if w_bz == 0:
        d_qb = np.array([0, v_bx, v_by])
    else:
        d_qb = np.array([w_bz, 
                        (v_bx*np.sin(w_bz) + v_by*(np.cos(w_bz) - 1))/w_bz,
                        (v_by*np.sin(w_bz) + v_bx*(1 - np.cos(w_bz)))/w_bz,
                        ])
    phi_k = curr_config[0]
    R_x = np.array([[1,     0,                      0],
                    [0, np.cos(phi_k), -np.sin(phi_k)],
                    [0, np.sin(phi_k),  np.cos(phi_k)],
                    ])
    d_q = R_x @ d_qb
    chass_new = chassis_cfg + d_q * dt
    new_config = np.concatenate((chass_new, new_arm, new_wheel), axis=None)
    new_config = new_config.tolist()
    return new_config
 

"""
Uncomment this code if you want to run it to generate the ms1.csv file
"""
# Assume everything is in nparrays
# dt = 0.01
# T = 100
# test_config = np.zeros([12])
# # print("Test", test_config)
# test_wheel_speeds = np.array([10, 10, 10, 10, 0,0,0,0,0])
# max_ang_speed = 10.0

# for_csv = [0,0,0,0,0,0,0,0,0,0,0,0,0]

# for i in range(T):
#     gripper_open = 0
#     test_config = NextState(test_config, test_wheel_speeds, dt, max_ang_speed)
#     gripper_flag = np.hstack((test_config, 0))
#     for_csv = np.vstack((for_csv, gripper_flag)) 
    # to_csv = np.append(to_csv, 0)

# np.savetxt('ms1.csv', for_csv, delimiter=",")