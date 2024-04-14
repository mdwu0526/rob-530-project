import csv
from InEkf import *
import matplotlib.pyplot as plt
from processData import *

alphas = np.square(np.array([0.00025, 0.00005, 0.0025, 0.0005, 0.0025, 0.0005]))

def main():
    csv_filename = 'log7.csv'
    data = loadData(csv_filename)
    data = processData(data)
    init_x = data['odometry x'][0]
    init_y = data['odometry y'][0]
    init_theta = data['odometry theta'][0]
    init_state = np.array([init_x, init_y, init_theta])
    filter = InEKF(init_state)

    curr_time = 0
    last_time = 0
    state_log = np.zeros((2,1))
    odom_log = np.zeros((2,1))
    landmarks = {}
    landmarks[1] = np.array([3.68, 1.8])
    landmarks[3] = np.array([3.12, 1.8])
    landmarks[5] = np.array([1.4, 6.24])
    landmarks[6] = np.array([1.4, 5.68])
    landmarks[7] = np.array([-0.28, 2.4])
    landmarks[8] = np.array([0.28, 2.4])
    for i in range(1,len(data['utime'])):
        print(i)
        last_time = data['utime'][i-1]
        curr_time = data['utime'][i]
        u = np.array([data['vel vx'][i], data['vel wz'][i]])
        dt = (curr_time - last_time)
        trans_vel = u[0]
        angular_vel = u[1]
        filter.prediction(u, dt)

        if data['apriltag 1 id'][i] != -1 and data['apriltag 2 id'][i] != -1:
            z = [data['apriltag 1 id'][i], data['apriltag 1 longitude'][i], data['apriltag 1 latitude'][i], data['apriltag 2 id'][i], data['apriltag 2 longitude'][i], data['apriltag 2 latitude'][i]]
            filter.correction(z, landmarks)
        state = state_vect(filter.mu)
        # print(state)
        if state_log.shape[1] == 0:
            state_log[:] = state[0:2]
            odom_log[:] = np.array([data['odometry x'][i], data['odometry y'][i]])
        else:
            state_log = np.hstack((state_log, state[0:2].reshape(-1,1)))
            odom_log = np.hstack((odom_log, np.array([data['odometry x'][i], data['odometry y'][i]]).reshape(-1,1)))
        last_time = curr_time
        plt.cla()
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
        plt.xlim(-2, 5)  # Setting x-axis limits from 0 to 6
        plt.ylim(-1, 8)  # Setting y-axis limits from 0 to 12
        plt.legend(['State', 'Odom'])
        plt.savefig('./IMG/{}.png'.format(i))

if __name__ == '__main__':
    main()