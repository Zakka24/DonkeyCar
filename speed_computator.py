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

# Computes Speed from Optitrack data
class SpeedComputator:
    def __init__(self, cfg, debug=True) -> None:
        # Init attributes
        self.old_pos = np.array([0.0, 0.0, 0.0])
        self.pos = np.array([0.0, 0.0, 0.0])
        self.speed = 0.0
        self.ewma = 0.0

        self.debug = debug
        self.cfg = cfg
        
        self.last_time = time.time()

        #self.print_velocity = not self.cfg.HAVE_ODOM    #if there is no encoder, the optitrack speed is shown on terminal
        self.print_velocity = False
        self.first = True


    def compute_speed(self) -> None:
        '''
        Compute speed every dt seconds.
        The DRIVE_LOOP period should be smaller than dt
        '''
        
        # Time 
        now = time.time()
        dt = now - self.last_time
        self.last_time = now      
        # Speed
        ds = np.subtract(self.pos, self.old_pos)
        self.speed = float(np.linalg.norm(ds) / dt)
        self.ewma = (ALPHA * self.speed) + ((1 - ALPHA) * self.ewma)

    def run(self, receiver_position: 'np.ndarray') -> float:
        # Modify positions
        # Save (z, y, x) in (x, y, z) (Reverse)
        
        for i in range(3):
            self.pos[i] = receiver_position[3-i-1]

        if self.debug:
            print(f"Speed computed by {__name__}: {self.speed} m/s\n")

        if self.print_velocity:
            stdout.write("\rVelocity (m/s): {:.3f}".format(self.speed))
            stdout.flush()
        
        if not self.first:
            self.compute_speed()
        else:
            self.first = False

        for i in range(3):
            self.old_pos[i] = self.pos[i]

        return self.ewma

