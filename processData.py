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
        data['odometry x'][i] = float(data['odometry x'][i]) if data['odometry x'][i] != "None" else None
        data['odometry y'][i] = float(data['odometry y'][i]) if data['odometry y'][i] != "None" else None
        data['odometry theta'][i] = float(data['odometry theta'][i]) if data['odometry theta'][i] != "None" else None
        data['vel vx'][i] = float(data['vel vx'][i]) if data['vel vx'][i] != "None" else None
        data['vel vy'][i] = float(data['vel vy'][i]) if data['vel vy'][i] != "None" else None
        data['vel wz'][i] = float(data['vel wz'][i]) if data['vel wz'][i] != "None" else None
        data['apriltag 1 longitude'][i] = float(data['apriltag 1 longitude'][i])
        data['apriltag 1 latitude'][i] = -1*float(data['apriltag 1 latitude'][i])
        data['apriltag 2 longitude'][i] = float(data['apriltag 2 longitude'][i])
        data['apriltag 2 latitude'][i] = -1*float(data['apriltag 2 latitude'][i])
        
    return data
    
def mergeData(folder):
    odom_data = loadData(folder + "/log_output_odom.csv")
    vel_data = loadData(folder + "/log_output_vel.csv")
    apriltag_data = loadData(folder + "/log_output_apriltag.csv")
    headers = ["utime", "vel vx", "vel vy", "vel wz", 
               "apriltag 1 id", "apriltag 1 latitude", "apriltag 1 longitude",
               "apriltag 2 id", "apriltag 2 latitude", "apriltag 2 longitude", 
               "odometry x", "odometry y", "odometry theta"]
    with open(folder + "/log_merged.csv", mode='w', newline='') as file:
        writer = csv.writer(file)

        # set iterators
        odom_i = 0
        vel_i = 0
        apriltag_i = 0

        # write first row of headers
        writer.writerow(headers)
        while(odom_i < len(odom_data['utime']) - 2 or vel_i < len(vel_data['utime']) - 2 or apriltag_i < len(apriltag_data['utime'])-1):
            # write velocity
            if(vel_data['utime'][vel_i] <= apriltag_data['utime'][apriltag_i] 
               and vel_data['utime'][vel_i] <= odom_data['utime'][odom_i]):
                row = [vel_data['utime'][vel_i], 
                        vel_data['vel vx'][vel_i], 
                        vel_data['vel vy'][vel_i], 
                        vel_data['vel wz'][vel_i],
                        -1,
                        -1,
                        -1,
                        -1,
                        -1,
                        -1,
                        "None",
                        "None",
                        "None"]
                if vel_data['utime'][vel_i] == odom_data['utime'][odom_i]:
                    row[10] = odom_data["odometry x"][odom_i]
                    row[11] = odom_data["odometry y"][odom_i]
                    row[12] = odom_data["odometry theta"][odom_i]
                    odom_i += 1
                writer.writerow(row)
                vel_i += 1

            # write apriltag
            elif(apriltag_i < len(apriltag_data['utime']) - 1
                 and apriltag_data['utime'][apriltag_i] < vel_data['utime'][vel_i]
                 and apriltag_data['utime'][apriltag_i] < odom_data['utime'][odom_i]):
                if(apriltag_data['utime'][apriltag_i] == apriltag_data['utime'][apriltag_i + 1]):
                    writer.writerow([apriltag_data['utime'][apriltag_i], 
                                 "None", 
                                 "None", 
                                 "None",
                                 apriltag_data['apriltag id'][apriltag_i],
                                 apriltag_data['apriltag x'][apriltag_i],
                                 apriltag_data['apriltag z'][apriltag_i],
                                 apriltag_data['apriltag id'][apriltag_i+1],
                                 apriltag_data['apriltag x'][apriltag_i+1],
                                 apriltag_data['apriltag z'][apriltag_i+1],
                                 "None",
                                 "None",
                                 "None"])
                    apriltag_i += 2
                else:
                    apriltag_i += 1
                if apriltag_i >= len(apriltag_data['utime'])-1:
                    apriltag_data['utime'][apriltag_i] = '9999999999999999'
            # write odom
            elif(odom_data['utime'][odom_i] < vel_data['utime'][vel_i]
                 and odom_data['utime'][odom_i] < apriltag_data['utime'][apriltag_i]):
                writer.writerow([apriltag_data['utime'][apriltag_i], 
                                 None, 
                                 None, 
                                 None,
                                 -1,
                                 -1,
                                 -1,
                                 -1,
                                 -1,
                                 -1,
                                 odom_data["odometry x"][odom_i],
                                 odom_data["odometry y"][odom_i],
                                 odom_data["odometry theta"][odom_i]])
                odom_i += 1

def getLandmarks(file):
    data = loadData(file)
    landmarks = {}
    for i in range(len(data['id'])):
        landmarks[int(data['id'][i])] = np.array([float(data['x'][i]), float(data['y'][i])])
    return(landmarks)

mergeData("data/log12")