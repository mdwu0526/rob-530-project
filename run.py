import csv
from InEkf import *
import matplotlib.pyplot as plt
from processData import *

alphas = np.square(np.array([0.00025, 0.00005, 0.0025, 0.0005, 0.0025, 0.0005]))

def main():
    # # get first odom
    # with open("data/log_output_odom.csv", 'r') as file:
    #     csv_reader = csv.reader(file)
    #     first_line = next(csv_reader)
    #     second_line = next(csv_reader)
    #     init_x = float(second_line[2])
    #     init_y = float(second_line[3])
    #     init_theta = float(second_line[3])
    #     init_state = np.array([init_x, init_y, init_theta])
            
    # filter = InEKF(init_state)

    # with open("data/log_output_vel.csv", 'r') as file:
    #     state_log = np.zeros((2,1))
    #     csv_reader = csv.reader(file)
    #     i = 0
    #     curr_time = 0
    #     last_time = 0
    #     for row in csv_reader:
    #         if i > 0:
    #             curr_time = float(row[0])
    #         if i > 1:
    #             u = np.array([float(row[2]), float(row[4])])
    #             dt = (curr_time - last_time) / 1e6
    #             trans_vel = u[0]
    #             angular_vel = u[1]
    #             noisy_motion = np.zeros(2)
    #             # noisy_motion[0] = np.random.normal(u[0], np.sqrt(alphas[0]*trans_vel**2+alphas[1]*angular_vel**2))
    #             # noisy_motion[1] = np.random.normal(u[1], np.sqrt(alphas[2]*trans_vel**2+alphas[3]*angular_vel**2))
    #             noisy_motion[0] = np.random.normal(u[0], np.sqrt(0.1))
    #             noisy_motion[1] = np.random.normal(u[1], np.sqrt(0.1))
    #             filter.prediction(noisy_motion, dt)
    #             state = state_vect(filter.mu)
    #             print(state)
    #             if state_log.shape[1] == 0:
    #                 state_log[:] = state[0:2]
    #             else:
    #                 state_log = np.hstack((state_log, state[0:2].reshape(-1,1)))
    #         last_time = curr_time
    #         i += 1
    #     print(state_log.shape)
    #     plt.plot(state_log[0,:], state_log[1,:])
    #     plt.show()
    csv_filename = 'log_5.csv'
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
    for i in range(1,len(data['utime'])):
        last_time = data['utime'][i-1]
        curr_time = data['utime'][i]
        u = np.array([data['vel vx'][i], data['vel wz'][i]])
        dt = (curr_time - last_time)
        trans_vel = u[0]
        angular_vel = u[1]
        noisy_motion = np.zeros(2)
        # noisy_motion[0] = np.random.normal(u[0], np.sqrt(alphas[0]*trans_vel**2+alphas[1]*angular_vel**2))
        # noisy_motion[1] = np.random.normal(u[1], np.sqrt(alphas[2]*trans_vel**2+alphas[3]*angular_vel**2))
        noisy_motion[0] = np.random.normal(u[0], np.sqrt(0.001))
        noisy_motion[1] = np.random.normal(u[1], np.sqrt(0.001))
        filter.prediction(noisy_motion, dt)
        filter.correction()
        state = state_vect(filter.mu)
        print(state)
        if state_log.shape[1] == 0:
            state_log[:] = state[0:2]
            odom_log[:] = np.array([data['odometry x'][i], data['odometry y'][i]])
        else:
            state_log = np.hstack((state_log, state[0:2].reshape(-1,1)))
            odom_log = np.hstack((odom_log, np.array([data['odometry x'][i], data['odometry y'][i]]).reshape(-1,1)))
        last_time = curr_time
    plt.plot(state_log[0,:], state_log[1,:])
    plt.plot(odom_log[0,:], odom_log[1,:])
    plt.show()


if __name__ == '__main__':
    main()