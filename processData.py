import csv
import numpy as np
        
def loadData(filename):        
    with open(filename, mode = 'r', newline = '', encoding='utf-8-sig') as file:
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
        data['apriltag 1 id'][i] = int(data['apriltag 1 id'][i])
        data['apriltag 2 id'][i] = int(data['apriltag 2 id'][i])
        data['odometry x'][i] = float(data['odometry x'][i])
        data['odometry y'][i] = float(data['odometry y'][i])
        data['odometry theta'][i] = float(data['odometry theta'][i])
        data['vel vx'][i] = float(data['vel vx'][i])
        data['vel vy'][i] = float(data['vel vy'][i])
        data['vel wz'][i] = float(data['vel wz'][i])
        data['apriltag 1 longitude'][i] = float(data['apriltag 1 longitude'][i])
        data['apriltag 1 latitude'][i] = float(data['apriltag 1 latitude'][i])
        data['apriltag 2 longitude'][i] = float(data['apriltag 2 longitude'][i])
        data['apriltag 2 latitude'][i] = float(data['apriltag 2 latitude'][i])
        
    return data
    
def main():
    csv_filename = 'log_output.csv'
    data = loadData(csv_filename)
    data = processData(data)
    
    return
    
if __name__ == '__main__':
    main()
        
        
        
        