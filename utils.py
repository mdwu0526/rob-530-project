import numpy as np

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