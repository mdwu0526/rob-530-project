import csv
from InEkf import *
from processData import *
from utils import *

ANIMATE = False
LOG = "log7"

def main():
    csv_filename = 'data/' + LOG + '/log_merged.csv'
    data = loadData(csv_filename)
    data = processData(data)
    init_state = getInitialState(data)
    landmarks = getLandmarks('data/' + LOG + '/landmarks.csv')
    filter = InEKF(init_state)

    curr_time = 0
    last_time = 0
    state_log = np.zeros((2,1))
    odom_log = np.zeros((2,1))
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

        if data['odometry x'][i] != None:
            odom = np.array([data['odometry x'][i], data['odometry y'][i]])
            if odom_log.shape[1] == 0:
                odom_log[:] = odom
            else:
                odom_log = np.hstack((odom_log, odom.reshape(-1,1)))

        
        plotScene(landmarks, state_log, odom_log, filter, i, len(data['utime']), ANIMATE)
        
if __name__ == '__main__':
    main()