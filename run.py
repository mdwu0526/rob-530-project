import numpy as np
import matplotlib
from processData import *
from RobotSystem import *

def main():
    
    data = Data()
    
    robot_system = RobotSystem(data)
    
    robot_system.run_filter()
    

if __name__ == '__main__':
    main()