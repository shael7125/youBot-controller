import core as c
import numpy as np

# function that computes and returns F based on dimensions of wheeled mobile base
def getF(l, w, r):
    H  = np.array([[-l-w, 1, -1],
                   [l+w, 1, 1],
                   [l+w, 1, -1],
                   [-l-w, 1, 1]])
    H = np.divide(H, r)
    # print(H)
    F = np.linalg.pinv(H)
    return F

def NextState(config, controls, dt, maxw):

    # Index config (12-vector) to get current chassis, arm, and wheel states
    # Assume all configs given in space frame, read in as row vectors
    chassis = config[0:3]
    arm = config[3:8]
    wheel = config[8:12]

    # Index controls (9-vector) to get wheel speeds and arm joint speeds
    wheelv = controls[0:4]
    armv = controls[4:]

    # Adjust speeds based on joint speed limits
    for entry in wheelv.flatten():
        if entry > maxw:
            entry = maxw
        elif entry < -maxw:
            entry = -maxw
    for entry in armv.flatten():
        if entry > maxw:
            entry = maxw
        elif entry < -maxw:
            entry = -maxw

    # use odometry to compute new chassis configuration
    # Compute H matrix and pseudoinvert to get F matrix
    
    # Compute F using mobile base's parameters
    l = 0.235
    w = 0.15
    r = 0.0475

    F = getF(l, w, r)

    # compute planar twist Vb (3 vector)
    # describes twist needed to advance config to next timestep
    # wheelv must by 3 x 1

    Vb = F @ (wheelv*dt)

    Vb6 = np.vstack((0, 0, Vb[0], Vb[1], Vb[2], 0))

    # compute transformation matrix Tbk,bk+1
    Tbkbkplus1 = c.MatrixExp6(c.VecTose3(Vb6.T.flatten()))

    # get Tsbk
    phi = chassis[0]
    x = chassis[1]
    y = chassis[2]
    # rotation is only about the z axis, use phi
    # translation is only about x and y axes
    Tsbk = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                     [np.sin(phi), np.cos(phi), 0, y],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    T = Tsbk @ Tbkbkplus1
    # print(T)

    phinew = np.atan2(T[1,0],T[0,0])

    # # update arm joint and wheel angles (Euler method) 
    wheelnew = wheel + dt*wheelv
    armnew = arm + dt*armv

    confignew = np.hstack((phinew, T[0,3], T[1,3], armnew, wheelnew))
    return confignew

# # trying controls 10, 10, 10, 10

config = np.zeros(12)
controls = np.array([10, 10, 10, 10, 0, 0, 0, 0, 0])
dt = 0.01
maxw = 100

drive = np.hstack((config, 0))
drive = drive.reshape(1,-1)


for i in range(100):
    # use the last row in drive array as input to NextState (omit gripper state)
    array = NextState(drive[-1,:12], controls, dt, maxw)
    # append a zero to array (for gripper state)
    array = np.hstack((array, 0))
    drive = np.vstack((drive,array))

# write list of sequentially updated configs to csv
np.savetxt("drive4.csv", np.asarray(np.c_[drive]), delimiter = ",")

