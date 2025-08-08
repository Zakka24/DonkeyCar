#!python3                                                                                                                                                                                               
  
import time
import json
import numpy as np
from sys import stdout

### CONSTANTS ###
MAX_SPEED: float = 3.0
ALPHA: float = 0.15
#ALPHA: float = 1.0


# KEYS
POS_KEY: str = 'position'
ROT_KEY: str = 'rotation'

### MAIN CLASS ###

# Computes Acceleration from Optitrack data
class AccelerationComputator:
    def __init__(self, cfg, debug=True) -> None:
        # Init attributes
        self.old_pos = np.array([0.0, 0.0, 0.0])
        self.pos = np.array([0.0, 0.0, 0.0])
        self.old_speed = 0.0
        self.current_speed = 0.0
        self.acceleration = 0.0
        self.ewma = 0.0

        self.acceleration_history = []
        self.ewma_history = []
        self.time_history = []
        self.start_time = time.time()

        self.debug = False
        self.cfg = cfg
        
        self.last_time = time.time()

        self.print_acceleration = False
        self.first = False


    def compute_acceleration(self) -> None:
        '''
        Compute acceleration every dt seconds.
        The DRIVE_LOOP period should be smaller than dt
        '''
        
        # Time 
        now = time.time()
        dt = now - self.last_time
        self.last_time = now      
        # Speed
        ds = np.subtract(self.pos, self.old_pos)
        self.old_speed = self.current_speed
        self.current_speed = float(np.linalg.norm(ds) / dt)
        self.acceleration = float((self.current_speed - self.old_speed) / dt)
        self.ewma = (ALPHA * self.acceleration) + ((1 - ALPHA) * self.ewma)
        
        current_time = now - self.start_time
        self.acceleration_history.append(self.acceleration)
        self.ewma_history.append(self.ewma)
        self.time_history.append(current_time)

    def run(self, receiver_position: 'np.ndarray') -> float:
        # Modify positions
        # Save (z, y, x) in (x, y, z) (Reverse)
        
        for i in range(3):
            self.pos[i] = receiver_position[3-i-1]

        if self.debug:
            print(f"Acceleration computed by {__name__}: {self.acceleration} m/s\n")
            #print(f"acceleration history: {self.acceleration_history}")
            #print(f"ewma acceleration: {self.ewma_history}")
            #print(f"time history: {self.time_history}")
            
        if self.print_acceleration:
            stdout.write("\Acceleration (m/s^2): {:.3f}".format(self.acceleration))
            stdout.flush()
        
        if not self.first:
            self.compute_acceleration()
        else:
            self.first = False

        for i in range(3):
            self.old_pos[i] = self.pos[i]

        return self.ewma
