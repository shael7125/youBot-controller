import core as c
import numpy as np
from milestone1 import getF

# # Test inputs (given_)

Tse = np.array([[0.170, 0, 0.985, 0.387],
                [0, 1, 0, 0],
                [-0.985, 0, 0.170, 0.570],
                [0, 0, 0, 1]])

Tsed = np.array([[0, 0, 1, 0.5],
                [0, 1, 0, 0],
                [-1, 0, 0, 0.5],
                [0, 0, 0, 1]])

Tsednext = np.array([[0, 0, 1, 0.6],
                [0, 1, 0, 0],
                [-1, 0, 0, 0.3],
                [0, 0, 0, 1]])

Kp = np.zeros([6,6])
Ki = Kp
dt = 0.01

Tsb = np.array([[np.cos(phi), -1*np.sin(phi), 0, config[1]],
                [np.sin(phi), np.cos(phi), 0, config[2]],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]])
Tb0 = np.array([[1, 0, 0, 0.1662],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0.0026],
                 [0, 0, 0, 1]])

M0e = np.array([[1, 0,  0, 0.033],
              [0, 1, 0, 0],
              [0, 0, 1, 0.6546],
              [0, 0,  0, 1]])
Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T

config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
thetalist = config[3:]
phi = config[0]

# use joint angles to compute end effector configuration (for jacobian)
T0e = c.FKinBody(M0e, Blist, thetalist)

# compute Jacobian
F = getF(0.235, 0.15, 0.0475)
zerom = np.zeros([1,4])
F6 = np.vstack([zerom, zerom, F, zerom])

Jbase = c.Adjoint(c.TransInv(T0e)@c.TransInv(Tb0))@F6
Jarm = c.JacobianBody(Blist, thetalist)
Je = np.hstack([Jbase, Jarm])

# initialize error sum (for I control)
errorsum = np.zeros((6,))
def FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, errorsum):
    
    # compute Xerr and Vd
    Xerr = c.se3ToVec(c.MatrixLog6(c.TransInv(Tse)@Tsed))
    Vd = c.se3ToVec(np.divide(c.MatrixLog6(c.TransInv(Tsed)@Tsednext),dt))
    
    # increment sum of the error
    errorsum = errorsum + Xerr*dt
    
    # compute error twist Ve(t)
    Adj = c.Adjoint(c.TransInv(Tse)@Tsed)
    V = Adj@Vd + Kp@Xerr +Ki@errorsum

    return V, Xerr, errorsum

Ve, Xerr, errorsum = FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt,errorsum)