
from mimetypes import init
from os import stat
from statistics import mean
from scipy.linalg import block_diag
from copy import deepcopy, copy
import numpy as np

# import InEKF lib
from scipy.linalg import logm, expm

from utils import *


class InEKF:
    def __init__(self, init_state):
        self.W = np.diag([0.25**2,0.25**2,0.5**2])     # motion noise covariance
        self.V = 5*np.eye(2)                      # measurement noise covariance
        
        self.mu = pose_mat(init_state)           # init_state will be the first odometry pose
        # SE(2) State
        # cos(theta)  -sin(theta)     x
        # sin(theta)   cos(theta)     y
        #     0           0           1
        self.Sigma = np.diag([1, 1, 1])
        # Covariance Matrix
        # var(x)      0       0
        #   0      var(y)     0
        #   0        0     var(theta)
    
    
    def prediction(self, u, dt):
        """
        u: control input. Given as [linear velocity, angular velocity]
        dt: timestep - should be measured via last time to current time
        """
        state_vector = state_vect(self.mu)

        # X state in SE(2) before prediction
        X_prev = self.mu
        
        # Apply nonlinear system dynamics to get the predicted state vector
        state_pred = self.gfun(state_vector, u, dt)
        
        # X_pred = state in SE(2) after prediction
        X_pred = pose_mat(state_pred)

        # Right Invariant Error
        u_se2 = logm(np.linalg.inv(X_prev) @ X_pred) # twist

        AdjX =  X_prev @ u_se2 @ np.linalg.inv(X_prev)
        self.mu = self.mu @ expm(u_se2)
        self.Sigma = self.Sigma + AdjX @ self.W @ AdjX.T
        
        # return np.copy(self.mu), np.copy(self.Sigma)
        
    def correction(self, z, landmarks):
        """
        landmarks: dictionary(key=id, value=[x,y])
        z: [id1, long1, lat1, id2, long2, lat2]
        """
        ###############################################################################
        # TODO: Implement the correction step for InEKF                               #
        # Hint: save your corrected state and cov as X and self.Sigma                 #
        # Hint: you can use landmark1.getPosition()[0] to get the x position of 1st   #
        #       landmark, and landmark1.getPosition()[1] to get its y position        #
        ###############################################################################
        G1 = np.zeros((3,3))
        G1[0,2] = 1
        G2 = np.zeros((3,3))
        G2[1,2] = 1
        G3 = np.zeros((3,3))
        G3[0,1] = -1
        G3[1,0] = 1
        id1 = z[0]
        long1 = z[1]
        lat1 = z[2]
        id2 = z[3]
        long2 = z[4]
        lat2 = z[5]
        Y1 = np.array([np.arctan2(lat1, long1), np.sqrt((lat1/1000) ** 2 + (long1/1000) ** 2), 1])
        Y2 = np.array([np.arctan2(lat2, long2), np.sqrt((lat2/1000) ** 2 + (long2/1000) ** 2), 1])

        landmark_gt1 = landmarks[id1]     #[x_gt, y_gt]
        landmark_gt2 = landmarks[id2] 

        b1 = np.array([landmark_gt1[0], landmark_gt1[1],1])
        H1 = np.array([[-1,0,landmark_gt1[1]],
                      [0,-1,-landmark_gt1[0]]])

        b2 = np.array([landmark_gt2[0], landmark_gt2[1],1])
        H2 = np.array([[-1,0,landmark_gt2[1]],
                      [0,-1,-landmark_gt2[0]]])
        
        b = np.hstack((b1,b2)) # 1x6
        H = np.vstack((H1,H2)) # 4x3
        
        Y = np.hstack((Y1,Y2)) # 
        # nu = self.mu @ Y.T - b # b = 2x3, self.mu = 3x3, Y.T = 2x2

        N = self.mu @ block_diag(self.V,0) @ self.mu.T # 3x3
        N = block_diag(N[0:2, 0:2], N[0:2, 0:2])
        S = H @ self.Sigma @ H.T + N
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        nu = block_diag(self.mu, self.mu) @ Y - b
        nu = np.hstack((nu[0:2],nu[3:5]))
        
        delta = K @ nu

        self.mu = np.dot(expm(delta[0] * G1 + delta[1] * G2 + delta[2] * G3), self.mu)
        self.Sigma = (np.eye(3) - K @ H) @ self.Sigma @ (np.eye(3) - K @ H).T + K @ N @ K.T

        ###############################################################################
        #                         END OF YOUR CODE                                    #
        ###############################################################################
        # self.state_.setState(X)
        # self.state_.setCovariance(self.Sigma)
        # return np.copy(X), np.copy(self.Sigma), np.copy(self.mu)

    def getState(self):
        return deepcopy(self.state_)

    def setState(self, state):
        self.state_ = state

    def gfun(self, mu, u, dt):
        k = 0.0000000000000000001
        u *= dt
        output = np.zeros(3)
        output[0] = mu[0] + (-u[0] / (u[1] + k) * np.sin(mu[2]) + u[0] / (u[1] + k) * np.sin(mu[2] + u[1]))
        output[1] = mu[1] + ( u[0] / (u[1] + k) * np.cos(mu[2]) - u[0] / (u[1] + k) * np.cos(mu[2] + u[1]))
        output[2] = mu[2] + u[1]
        return output