import core as c
import numpy as np

from milestone1 import getF
from milestone1 import NextState

from milestone2 import TrajectoryGenerator
from milestone3 import FeedbackControl

# # FOR BEST
kp = 1.75
ki = 1.5
config = np.array([np.pi/6, 0, 0.2, 0, np.pi/3, -np.pi/4, -np.pi/2, 0, 0, 0, 0, 0])
Tsc0 = np.array([[1, 0, 0, 1],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]])
Tscf = np.array([[0, 1, 0, 0],
                 [-1, 0, 0, -1],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]])


# # FOR OVERSHOOT
# kp = 5
# ki = 50
# config = np.array([np.pi/6, 0, 0.2, 0, np.pi/3, -np.pi/4, -np.pi/2, 0, 0, 0, 0, 0])
# Tsc0 = np.array([[1, 0, 0, 1],
#                  [0, 1, 0, 0],
#                  [0, 0, 1, 0.025],
#                  [0, 0, 0, 1]])
# Tscf = np.array([[0, 1, 0, 0],
#                  [-1, 0, 0, -1],
#                  [0, 0, 1, 0.025],
#                  [0, 0, 0, 1]])


# # FOR NEWCASE, 4 best
# kp = 4
# ki = 0
# config = np.array([np.pi/6, 0, 0.2, 0, np.pi/3, -np.pi/4, -np.pi/2, 0, 0, 0, 0, 0])
# # [phi, x, y] = [pi/4, 1, 1]
# Tsc0 = np.array([[1/np.sqrt(2), -1/np.sqrt(2), 0, 1],
#                  [1/np.sqrt(2), 1/np.sqrt(2), 0, 1],
#                  [0, 0, 1, 0.025],
#                  [0, 0, 0, 1]])
# # [phi, x, y] = [-pi/4, 1, -1]
# Tscf = np.array([[1/np.sqrt(2), 1/np.sqrt(2), 0, 1],
#                  [-1/np.sqrt(2), 1/np.sqrt(2), 0, -1],
#                  [0, 0, 1, 0.025],
#                  [0, 0, 0, 1]])

# # Testing FeedForward control: success! comment out Tse0 definition below (line 108)
# config = np.array([0, 0, 0, 0.1, -np.pi/4, 0.1, 0, np.pi/2, 0, 0, 0, 0])
# Tse0, _, _ = configdecomp(config)

# # # INITIALIZATIONS
# Given M0e, Blist, and Tb0
M0e = np.array([[1, 0,  0, 0.033],
              [0, 1, 0, 0],
              [0, 0, 1, 0.6546],
              [0, 0,  0, 1]])

Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T

Tb0 = np.array([[1, 0, 0, 0.1662],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0.0026],
                 [0, 0, 0, 1]])

# # Reference trajectory:

# end effector grasp config
Tcegrasp = np.array([[-0.5*np.sqrt(2), 0, 0.5*np.sqrt(2), 0],
                     [0, 1, 0, 0],
                     [-0.5*np.sqrt(2), 0, -0.5*np.sqrt(2), 0],
                     [0, 0, 0, 1]])

# end effector standoff config
Tcestandoff = np.array([[-0.5*np.sqrt(2), 0, 0.5*np.sqrt(2), 0],
                     [0, 1, 0, 0],
                     [-0.5*np.sqrt(2), 0, -0.5*np.sqrt(2), 0.05],
                     [0, 0, 0, 1]])

# configdecomp extracts phi, x, and y, and thetalist from a configuration row vector
# configdecomp computes and returns Tse and T0e, and returns thetalist

def configdecomp(config):
    phi = config[0]
    x = config[1]
    y = config[2]
    thetalist = config[3:8]
    Tsb = np.array([[np.cos(phi), -1*np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])
    T0e = c.FKinBody(M0e,Blist,thetalist)
    
    Tse = Tsb@Tb0@T0e

    return Tse, T0e, thetalist

# Given Tse0 as initial position along reference trajectory
Tse0 = np.array([[0, 0, 1, 0],
                 [0, 1, 0, 0],
                 [-1, 0, 0, 0.5],
                 [0, 0, 0, 1]])

# generate reference trajectory
TrajList, totaltime, gripper = TrajectoryGenerator(Tse0, Tsc0, Tscf, Tcegrasp, Tcestandoff, 1)

# # Initializations for Loop (NextState and FeedbackController)
dt = 0.01
maxw = 100

# initialize F6 for looped jacobian calculation
F = getF(0.235, 0.15, 0.0475)
zerom = np.zeros([1,4])
F6 = np.vstack([zerom, zerom, F, zerom])

# define initials before entering the loop
config = np.hstack((config, gripper[0]))
soln = config.reshape(1,-1)
error = np.empty((1,6))
errorsum = np.zeros((6,))

for i in range(len(TrajList)-1):

    # get end effector position and arm joint angles fron current configuration
    Tse, T0e, thetalist = configdecomp(config)

    # Jacobian calculation, depends on angles of robot
    Jbase = 5*c.Adjoint(c.TransInv(T0e)@c.TransInv(Tb0))@F6
    Jarm = c.JacobianBody(Blist, thetalist)
    Je = np.hstack([Jbase, Jarm])

    # Define inputs for Feedback Controller
    Tsed = TrajList[i]
    Tsednext = TrajList[i+1]
    Kp = kp*np.eye(6)
    Ki = ki*np.eye(6)

    # use FeedbackController to compute controls and increment errorsum
    Ve, Xerr, errorsum = FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, errorsum)
    controls = np.linalg.pinv(Je,rtol= 0.0001)@Ve
   
    # store error for plotting later
    error = np.vstack((error, Xerr))

    # use NextState to update the configuration
    config = NextState(config, controls, dt, maxw)
    config = np.hstack((config, gripper[i+1]))
    soln = np.vstack((soln, config))

# write solution and error to csv
np.savetxt("soln.csv", np.asarray(np.c_[soln]), delimiter = ",")
np.savetxt("error.csv", np.asarray(np.c_[error]), delimiter = ",")


