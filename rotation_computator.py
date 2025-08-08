#!python3

import quaternion
import numpy as np
from scipy.spatial.transform import Rotation

import logging
logger = logging.getLogger(__name__)

class RotationComputator:
    '''
    This part will translate Quaternion message into Euler angle
    rotation in a 2D space
    '''
    
    def __init__(self, debug = False):
        self.debug = debug

    def run_threaded(self, recv_q: float) -> float:
        return run(recv_q)

    def run(self, recv_q: 'np.array') -> float:  
        #if self.debug:
        #    print('-- Rotation Computator -- Recv Quaternion:', recv_q)
        
        q = quaternion.quaternion(
            recv_q[3], 
            recv_q[0], 
            recv_q[1], 
            recv_q[2]
        )
        #if self.debug:
        #    print('-- Rotation Computator -- Quaternion:', q)
        
        rotation = 0.0
        # Angle wrt x
        try:
            rotation = self.quaternion_to_euler(q)[0]
        except Exception as e:
            if self.debug:
                print('-- Rotation Computator --', e)
        
        if rotation <= -180:
            rotation += 360
        elif rotation > 180:
            rotation -= 360
        
        if self.debug:
            print('-- Rotation Computator -- Rotation:', rotation)
        return rotation # Return only y angle

    def quaternion_to_euler(self, q: float) -> list:
        #convert quaternion to list
        quat = [q.w, q.x, q.y, q.z]
        if np.linalg.norm(quat) == 0:
            raise ValueError("Zero norm quaternion.")

        rot = Rotation.from_quat(quat)
        return rot.as_euler('yzx', degrees=True)
