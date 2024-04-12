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
        self.W = np.diag([0.05**2,0.1**2,0.1**2])     # motion noise covariance
        self.V = 10000*np.eye(2)                      # measurement noise covariance
        
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
        
    # def correction(self, Y1, Y2, z, landmarks, mu_pred, sigma_pred):
    #     landmark1 = landmarks.getLandmark(z[2].astype(int))
    #     landmark2 = landmarks.getLandmark(z[5].astype(int))
    #     self.mu_pred = mu_pred
    #     self.sigma_pred = sigma_pred
    #     ###############################################################################
    #     # TODO: Implement the correction step for InEKF                               #
    #     # Hint: save your corrected state and cov as X and self.Sigma                 #
    #     # Hint: you can use landmark1.getPosition()[0] to get the x position of 1st   #
    #     #       landmark, and landmark1.getPosition()[1] to get its y position        #
    #     ###############################################################################
    #     b1 = np.array([landmark1.getPosition()[0], landmark1.getPosition()[1], 1])
    #     b2 = np.array([landmark2.getPosition()[0], landmark2.getPosition()[1], 1])
    #     b = np.hstack((b1, b2))
    #     Y = np.hstack((Y1, Y2))
    #     H_1 = np.array([[b1[1], -1, 0], [-b1[0], 0, -1]])
    #     H_2 = np.array([[b2[1], -1, 0], [-b2[0], 0, -1]])
    #     H = np.vstack((H_1, H_2))
    #     N = mu_pred @ block_diag(self.V, 0) @ mu_pred.T
    #     N = block_diag(N[0:2, 0:2], N[0:2, 0:2])
    #     S = H @ sigma_pred @ H.T + N
    #     L = sigma_pred @ H.T @ np.linalg.inv(S)
        
    #     # Update state
    #     nu = block_diag(mu_pred, mu_pred) @ Y - b
    #     nu = np.hstack((nu[0:2],nu[3:5]))
        
    #     def wedge(phi):
    #         """
    #         R^3 vector to so(3) matrix
    #         @param  phi: R^3
    #         @return Phi: so(3) matrix
    #         """
    #         G1 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
    #         G2 = np.array([[0, 0, 1], [0, 0, 0], [0, 0, 0]])
    #         G3 = np.array([[0, 0, 0], [0, 0, 1], [0, 0, 0]])
    #         Phi = G1 * phi[2] + G2 * phi[0] + G3 * phi[1]
    #         return Phi
        
    #     delta = wedge(L @ nu)
        
    #     self.mu = expm(delta) @ mu_pred
        
    #     X = np.array([self.mu[0, 2], self.mu[1, 2], wrap2Pi(np.arctan2(self.mu[1, 0], self.mu[0, 0]))])
        
    #     # Update covariance
    #     self.Sigma = (np.eye(3) - L @ H) @ self.Sigma @ (np.eye(3) - L @ H).T + L @ N @ L.T

    #     ###############################################################################
    #     #                         END OF YOUR CODE                                    #
    #     ###############################################################################
    #     self.state_.setState(X)
    #     self.state_.setCovariance(self.Sigma)
    #     return np.copy(X), np.copy(self.Sigma), np.copy(self.mu)

    def getState(self):
        return deepcopy(self.state_)

    def setState(self, state):
        self.state_ = state

    def gfun(self, mu, u, dt):
        u *= dt
        output = np.zeros(3)
        output[0] = mu[0] + (-u[0] / u[1] * np.sin(mu[2]) + u[0] / u[1] * np.sin(mu[2] + u[1]))
        output[1] = mu[1] + ( u[0] / u[1] * np.cos(mu[2]) - u[0] / u[1] * np.cos(mu[2] + u[1]))
        output[2] = mu[2] + u[1]
        return output