import numpy as np
from scipy.linalg import logm, expm
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
import cv2 as cv
import os
import math

def wrap2Pi(input):
    phases =  (( -input + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0

    return phases

def pose_mat(X):
        x = X[0]
        y = X[1]
        h = X[2]
        H = np.array([[np.cos(h),-np.sin(h),x],\
                      [np.sin(h),np.cos(h),y],\
                      [0,0,1]])
        return H
    
def state_vect(mu):
    return np.array([mu[0,2], mu[1,2], np.arctan2(mu[1,0], mu[0,0])])

def confidence_ellipse(X, L):
    # create confidence ellipse
    # se(2) Lie algebra basis twist = vec(v1, v2, omega)
    G1 = np.array([[0, 0, 1],
                    [0, 0, 0],
                    [0, 0, 0]])
    G2 = np.array([[0, 0, 0],
                    [0, 0, 1],
                    [0, 0, 0]])
    G3 = np.array([[0, -1, 0],
                    [1, 0, 0],
                    [0, 0, 0]])

    # first create points from a unit circle + angle (third dimension of so(3))
    phi = np.arange(-np.pi, np.pi+0.01, 0.01)
    circle = np.array([np.cos(phi), np.sin(phi), np.zeros(np.size(phi))]).T
    # Chi-squared 2-DOF 95% confidence (0.05): 7.815
    scale = np.sqrt(7.815)
    # main loop; iterate over the control inputs and move the robot
    ELLIPSE = np.zeros([np.shape(circle)[0], 2])  # covariance ellipse on manifold (nonlinear)
    for j in range(np.shape(circle)[0]):
        # sample covariance on SE(2)
        ell_se2_vec = scale * L @ circle[j,:].reshape(-1,1)
        # retract and left-translate the ellipse on Lie algebra to SE(2) using Lie exp map
        temp = np.dot(X, expm(G1 * ell_se2_vec[0] + G2 * ell_se2_vec[1] + G3 * ell_se2_vec[2]))
        ELLIPSE[j, :] = np.array([temp[0, 2], temp[1, 2]])
    return ELLIPSE

def getInitialState(data):
    for i in range(len(data["utime"])):
        if data["odometry x"][i] is not None:
            init_x = data['odometry x'][i]
            init_y = data['odometry y'][i]
            init_theta = data['odometry theta'][i]
            init_state = np.array([init_x, init_y, init_theta])
            return init_state
        
def plotScene(landmarks, state_log, odom_log, filter, i, data_len, ANIMATE):
    if ANIMATE or (i >= data_len-1):
        fig, ax = plt.subplots()
        plt.cla()
        gt = ax.plot([0, 1.32, 1.32, 0, 0], [0, 0, 1.04, 1.04, 0])
        states = ax.plot(state_log[0,:], state_log[1,:])
        wedge = Wedge(center=(state_log[0,i], state_log[1,i]), 
                      r=1, 
                      theta1 = math.degrees(state_vect(filter.mu)[2] - 2*np.sqrt(filter.Sigma[2,2])), 
                      theta2 = math.degrees(state_vect(filter.mu)[2] + 2*np.sqrt(filter.Sigma[2,2])),
                      color='green', 
                      alpha=0.5)
        ax.add_patch(wedge)
        ax.set_aspect('equal', 'box')
        odoms = ax.plot(odom_log[0,:], odom_log[1,:])
        for id in landmarks:
            lmrks = ax.scatter(landmarks[id][0], landmarks[id][1], color = 'hotpink', marker='*')
        ELLIPSE = confidence_ellipse(filter.mu, np.linalg.cholesky(filter.Sigma))
        plt.plot(ELLIPSE[:, 0], ELLIPSE[:, 1], color='red', alpha=0.7, linewidth=1.5)
        plt.quiver(filter.mu[0, 2], filter.mu[1, 2], 10 * filter.mu[0, 0], 10 * filter.mu[1, 0],color='darkblue')
        plt.xlim(-1, 2)
        plt.ylim(-1, 2)
        ax.legend([gt[0], states[0], odoms[0], lmrks], ["GT", "State", "Odometry", "Landmark"])
        plt.savefig('./IMG/{}.png'.format(i))
        if i >= data_len-1:
            if ANIMATE:
                make_avi()
            else:
                plt.show()
        plt.close()

def make_avi():
    print("Making AVI...")
    v_dir = 'riekf_localization_se2.avi'
    fps = 20
    img_t = cv.imread('./IMG/1.png')
    img_size = (img_t.shape[1], img_t.shape[0])

    files = os.listdir('./IMG')
    N = len(files)

    fourcc = cv.VideoWriter_fourcc('M', 'J', 'P', 'G')
    v_w = cv.VideoWriter(v_dir, fourcc, fps, img_size)

    for i in range(1, N):
        img = cv.imread('./IMG/{}.png'.format(i))
        v_w.write(img)

    v_w.release()
    print("Done.")