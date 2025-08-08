#!python3

import sys

### CONSTANTS ###
MAX_SPEED = float("inf")

### MAIN CLASS ###

# Computes Speed from Optitrack data
class SpeedSwitch:
    def __init__(self, manual: bool = True, lidar: bool = False, 
    v2v: bool = False, debug:bool = False) -> None:
        
        if not manual:
            print('Speed Switch --- WARNING! Manual Speed disabled')
        
        # Init attributes
        self.manual = manual
        self.lidar = lidar
        self.v2v = v2v
        self.debug = debug
        
        self.speeds = [0, 0, 0]
    
    def run_threaded(self, manual_speed: float,
    lidar_speed: float, v2v_speed: float) -> float:
        return self.run(manual_speed, lidar_speed, v2v_speed)

    def run(self, manual_speed: float,
    lidar_speed: float, v2v_speed: float) -> float:

        #print('V2V Speed:', v2v_speed)
        '''
        Return the minimum of all the input speeds
        '''
        self.speeds[0] = manual_speed if self.manual else 0.0 # For safety
        self.speeds[1] = lidar_speed if self.lidar else MAX_SPEED
        self.speeds[2] = v2v_speed if self.v2v else MAX_SPEED
        
        user_speed = min(self.speeds)

        if self.debug:
            chosen_speed = self.speeds.index(retval)
            if chosen_speed == 1:
                chosen_speed = 'Lidar'
            elif chosen_speed == 2:
                chosen_speed = 'V2V'
            else:
                chosen_speed = 'Manual'

            print('Speed Switch --- {} {:.3f}'.format(
                chosen_speed,
                retval
            ), end='\r')
            sys.stdout.flush()

        return user_speed

    
    def shutdown(self) -> None:
        pass

    def update(self) -> None:
        pass
