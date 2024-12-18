# # # CUT AND PASTE THIS COMMENT
# # create Tse0
# M0e = np.array([[1, 0,  0, 0.033],
#               [0, 1, 0, 0],
#               [0, 0, 1, 0.6546],
#               [0, 0,  0, 1]])


# Tb0 = np.array([[1, 0, 0, 0.1662],
#                  [0, 1, 0, 0],
#                  [0, 0, 1, 0.0026],
#                  [0, 0, 0, 1]])

# phi = 0
# x = 0
# y = 0
# z = 0
# Tsb = np.array([[np.cos(phi), -1*np.sin(phi), 0, x],
#                 [np.sin(phi), np.cos(phi), 0, y],
#                 [0, 0, 1, 0.0963],
#                 [0, 0, 0, 1]])

# Tse0 = Tsb@Tb0@M0e

# # Given: Tsc0 and Tscf
# Tsc0 = np.array([[1, 0, 0, 1],
#                  [0, 1, 0, 0],
#                  [0, 0, 1, 0.025],
#                  [0, 0, 0, 1]])
# Tscf = np.array([[0, 1, 0, 0],
#                  [-1, 0, 0, -1],
#                  [0, 0, 1, 0.025],
#                  [0, 0, 0, 1]])

# # Grasp configuration
# Tcegrasp = np.array([[-0.5*np.sqrt(2), 0, 0.5*np.sqrt(2), 0],
#                      [0, 1, 0, 0],
#                      [-0.5*np.sqrt(2), 0, -0.5*np.sqrt(2), 0],
#                      [0, 0, 0, 1]])

# # Standoff configuration
# Tcestandoff = np.array([[0, 0, 0, 0],
#                         [0, 0, 0, 0],
#                         [0, 0, 0, 0.05],
#                         [0, 0, 0, 0]])
# Tcestandoff = Tcestandoff + Tcegrasp

# k = 1

# TrajectoryGenerator(Tse0, Tsc0, Tscf, Tcegrasp, Tcestandoff, k)

import core as c
import numpy as np

# # The goal of this function is to generate a reference trajectory for the robot.
# # This will later be used for feedforward control

def TrajectoryGenerator(Tse0, Tsc0, Tscf, Tcegrasp, Tcestandoff, k):
    
    # Define the time that each of the 8 trajectories should take
    t1 = 7.51
    t2 = 3.43
    # t3 is set to take 0.63 seconds
    t4 = t2
    t5 = t1
    t6 = t2
    # t7 is set to take 0.63 seconds
    t8 = t2

    totaltime = t1*2+t2*4+0.63*2

    # Use 5th order polynomials for trajectory generation
    ord = 5

    #Define initial and final e.e. standoff and grasp configurations in the spaceframe
    Tsestandoff0 = Tsc0@Tcestandoff 
    Tsestandofff = Tscf@Tcestandoff
    Tsegrasp0 = Tsc0@Tcegrasp
    Tsegraspf = Tscf@Tcegrasp

    # Move gripper from initial configuration to initial standoff configuration (1)
    TrajList = c.ScrewTrajectory(Tse0, Tsestandoff0, t1, t1/(0.01/k), ord)

    # Move gripper from initial standoff to cube's initial position (2)
    TrajList.extend(c.CartesianTrajectory(Tsestandoff0,Tsegrasp0, t2, t2/(0.01/k), ord))
    len2 = len(TrajList)

    # close gripper (3)
    for j in range(63*k):
        TrajList.append(Tsegrasp0)
   
    # move gripper back to initial standoff configuration (4)
    TrajList.extend(c.CartesianTrajectory(Tsegrasp0, Tsestandoff0, t4, t4/(0.01/k), ord))
    
    # move gripper from initial standoff position to final standoff position (5)
    TrajList.extend(c.ScrewTrajectory(Tsestandoff0, Tsestandofff, t5, t5/(0.01/k), ord))
    
    # move gripper from final standoff position to cube's final position (6)
    TrajList.extend(c.CartesianTrajectory(Tsestandofff, Tsegraspf, t6, t6/(0.01/k), ord))
    len6 = len(TrajList)
    
    # open gripper (7)
    for j in range(63*k):
        TrajList.append(Tsegraspf)
    
    # move gripper back to final standoff configuration (8)
    TrajList.extend(c.CartesianTrajectory(Tsegraspf, Tsestandofff, t8, t8/(0.01/k), ord))

    # initialize gripper array
    gripper = np.zeros(len(TrajList))

    # initialize reftraj array
    reftraj = np.empty((0,13))


    for i in range(len(TrajList)): # loop through each transformation matrix in the list created above
        # extract rotation matrix and position vector
        rotmat = TrajList[i][:3,:3]
        posvec = TrajList[i][:3,3]
        # use metrics from above (when segment 3 begins to when segment 7 ends) to create gripper array
        if i > len2 and i <= len6:
            gripper[i] = 1
        currenttraj = np.hstack((rotmat.flatten(),posvec,[gripper[i]]))
        reftraj = np.vstack((reftraj,currenttraj))

    # save reference trajectory to csv file
    np.savetxt("reftraj.csv", np.asarray(np.c_[reftraj]), delimiter = ",")
    
    return TrajList, totaltime, gripper


# create Tse0
M0e = np.array([[1, 0,  0, 0.033],
              [0, 1, 0, 0],
              [0, 0, 1, 0.6546],
              [0, 0,  0, 1]])


Tb0 = np.array([[1, 0, 0, 0.1662],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0.0026],
                 [0, 0, 0, 1]])

phi = 0
x = 0
y = 0
Tsb = np.array([[np.cos(phi), -1*np.sin(phi), 0, x],
                [np.sin(phi), np.cos(phi), 0, y],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]])

Tse0 = Tsb@Tb0@M0e

# Given: Tsc0 and Tscf
Tsc0 = np.array([[1, 0, 0, 1],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]])
Tscf = np.array([[0, 1, 0, 0],
                 [-1, 0, 0, -1],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]])

# Grasp configuration
Tcegrasp = np.array([[-0.5*np.sqrt(2), 0, 0.5*np.sqrt(2), 0],
                     [0, 1, 0, 0],
                     [-0.5*np.sqrt(2), 0, -0.5*np.sqrt(2), 0],
                     [0, 0, 0, 1]])

# Standoff configuration
Tcestandoff = np.array([[0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0.05],
                        [0, 0, 0, 0]])
Tcestandoff = Tcestandoff + Tcegrasp

k = 1

TrajList = TrajectoryGenerator(Tse0, Tsc0, Tscf, Tcegrasp, Tcestandoff, k)
