import numpy as np
from scipy.linalg import logm, expm
import matplotlib.pyplot as plt
import cv2 as cv
import os

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
    # se(2) Lie algebra basis twist = vec(omega, v1, v2)
    G1 = np.array([[0, -1, 0],
                   [1, 0, 0],
                   [0, 0, 0]])
    G2 = np.array([[0, 0, 1],
                   [0, 0, 0],
                   [0, 0, 0]])
    G3 = np.array([[0, 0, 0],
                   [0, 0, 1],
                   [0, 0, 0]])

    # first create points from a unit circle + angle (third dimension of so(3))
    phi = np.arange(-np.pi, np.pi+0.01, 0.01)
    circle = np.array([np.zeros([len(phi), 1]), np.cos(phi).reshape(-1, 1), np.sin(phi).reshape(-1, 1)]).reshape(3, -1)
    # Chi-squared 2-DOF 95% confidence (0.05): 7.815
    scale = np.sqrt(7.815)
    # main loop; iterate over the control inputs and move the robot
    ELLIPSE = np.zeros([circle.shape[1], 2])  # covariance ellipse on manifold (nonlinear)
    for j in range(circle.shape[1]):
        # sample covariance on SE(2)
        ell_se2_vec = scale * np.dot(L, circle[:, j])
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
        plt.cla()
        plt.plot([0, 3.4, 3.4, 0, 0], [0, 0, 5.96, 5.96, 0])
        plt.plot(state_log[0,:], state_log[1,:])
        plt.plot(odom_log[0,:], odom_log[1,:])
        plt.scatter(landmarks[1][0], landmarks[1][1])
        plt.scatter(landmarks[3][0], landmarks[3][1])
        plt.scatter(landmarks[5][0], landmarks[5][1])
        plt.scatter(landmarks[6][0], landmarks[6][1])
        plt.scatter(landmarks[7][0], landmarks[7][1])
        plt.scatter(landmarks[8][0], landmarks[8][1])
        ELLIPSE = confidence_ellipse(filter.mu, np.linalg.cholesky(filter.Sigma))
        plt.plot(ELLIPSE[:, 0], ELLIPSE[:, 1], color='red', alpha=0.7, linewidth=1.5)
        plt.xlim(-2, 5)  # Setting x-axis limits from -2 to 5
        plt.ylim(-1, 8)  # Setting y-axis limits from -1 8
        plt.legend(['State', 'Odom'])
        plt.savefig('./IMG/{}.png'.format(i))
        if i >= data_len-1:
            if ANIMATE:
                make_avi()
            else:
                plt.show()

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