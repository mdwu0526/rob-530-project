import csv
import numpy as np
        
def loadData(filename):        
    with open(filename, mode = 'r', newline = '') as file:
        reader = csv.reader(file)
        headers = next(reader)
        out = {header: [] for header in headers}
        
        for row in reader:
            for header, value in zip(headers, row):
                out[header].append(value)
    
    return out

def processData(data):
    base_time = float(data['utime'][0])

    for i in range(len(data['utime'])):
        data['utime'][i] = (float(data['utime'][i]) - base_time) / 1000000
        data['apriltag id'][i] = int(data['apriltag id'][i])
        data['apriltag world_x'][i] = float(data['apriltag world_x'][i])
        data['apriltag world_y'][i] = float(data['apriltag world_y'][i])
        data['odometry x'][i] = float(data['odometry x'][i])
        data['odometry y'][i] = float(data['odometry y'][i])
        data['odometry theta'][i] = float(data['odometry theta'][i])
        data['vel vx'][i] = float(data['vel vx'][i])
        data['vel vy'][i] = float(data['vel vy'][i])
        data['vel wz'][i] = float(data['vel wz'][i])
        data['apriltag longitude'][i] = float(data['apriltag longitude'][i])
        data['apriltag latitude'][i] = float(data['apriltag latitude'][i])
        
    return data
    
def main():
    csv_filename = 'log_output.csv'
    data = loadData(csv_filename)
    data = processData(data)
    
    return
    
if __name__ == '__main__':
    main()
        
        
        
        