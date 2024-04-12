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
    data.pop('type')
    data.pop('apriltag y')
    base_time = float(data['utime'][0])

    for i in range(len(data)):
        data['utime'][i] = (float(data['utime'][i]) - base_time) / 1000000
        data['apriltag id'][i] = int(data['apriltag id'][i])
        data['apriltag x'][i] = float(data['apriltag x'][i])
        data['apriltag z'][i] = float(data['apriltag z'][i])
        data['odometry x'][i] = float(data['odometry x'][i])
        data['odometry y'][i] = float(data['odometry y'][i])
        data['odometry theta'][i] = float(data['odometry theta'][i])
        data['vel vx'][i] = float(data['vel vx'][i])
        data['vel vy'][i] = float(data['vel vy'][i])
        data['vel wz'][i] = float(data['vel wz'][i])
        
    return data
    
def main():
    csv_filename = 'log_output.csv'
    data = loadData(csv_filename)
    data = processData(data)
    
    return
    
if __name__ == '__main__':
    main()
        
        
        
        