import numpy as np
from scipy.linalg import logm, expm

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