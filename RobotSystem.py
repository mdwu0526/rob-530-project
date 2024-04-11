import numpy as np

class RobotSystem:
    
    def __init__(self, data=None):
        self.num_steps = data['time'].shape(0)
        
    def run_filter(self):
        
        results = np.zeros(self.num_steps)