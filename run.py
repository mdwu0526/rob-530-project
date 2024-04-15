import csv
from InEkf import *
import matplotlib.pyplot as plt
from processData import *
from utils import *

def main():
    csv_filename = 'data/log7/log_merged.csv'
    data = loadData(csv_filename)
    data = processData(data)
    init_state = getInitialState(data)
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
    landmarks[7] = np.array([-0.28, 5.96-2.4])
    landmarks[8] = np.array([0.28, 5.96-2.4])
    for i in range(0,len(data['utime'])):
        print(i)
        if data['vel vx'][i] != None:
            curr_time = data['utime'][i]
            if (last_time != 0):
                u = np.array([data['vel vx'][i], data['vel wz'][i]])
                dt = (curr_time - last_time)
                filter.prediction(u, dt)
            last_time = curr_time

        if data['apriltag 1 id'][i] != -1 and data['apriltag 2 id'][i] != -1:
            z = [data['apriltag 1 id'][i], data['apriltag 1 longitude'][i], data['apriltag 1 latitude'][i], data['apriltag 2 id'][i], data['apriltag 2 longitude'][i], data['apriltag 2 latitude'][i]]
            filter.correction(z, landmarks)
        state = state_vect(filter.mu)

        if state_log.shape[1] == 0:
            state_log[:] = state[0:2]
        else:
            state_log = np.hstack((state_log, state[0:2].reshape(-1,1)))

    plt.cla()
    plt.plot(state_log[0,:], state_log[1,:])
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
    plt.show()

if __name__ == '__main__':
    main()