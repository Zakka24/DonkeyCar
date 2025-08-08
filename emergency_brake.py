#!python3

import sys

### CONSTANTS ###
MAX_THROTTLE: float = 0.25
#################

### MAIN CLASS ###

# Computes Speed from Optitrack data
class EmergencyBrake:
    '''
    :param manual Use manual brake
    :param lidar Use lidar stop signal
    :param v2v Use v2v stop signal
    :param limit_space True if you want to limit car position
            within the limits in the param border
    :param borders Position borders of the car:
        borders = [x_left, x_right, y_left, y_right]
    '''
    def __init__(self, manual: bool = True, lidar: bool = False, 
    v2v: bool = False, limit_space: bool = False, 
    borders: list = [-2.0, 2.0, -2.0, 2.0], debug: bool = False) -> None:
        
        if not manual:
            print('Emergency Brake --- WARNING! Manual Stop disabled')

        # Init attributes
        self.manual = manual
        self.lidar = lidar
        self.v2v = v2v
        self.limit_space = limit_space
        self.borders = borders
        self.debug = debug
    '''
    :param user_throttle Ordinal pwm value
    :param manual_stop Stop manually triggered
    :param lidar_stop Stop triggered by Lidar
    :param v2v_stop Stop triggered by v2v
    :param car_pos Position of car:
        car_pos = [x_cor, y_cor]
    '''
    def run_threaded(self, user_throttle: float, manual_stop: bool,
    lidar_stop: bool, v2v_stop: bool, car_pos: list) -> float:
        return self.run(user_throttle, manual_stop, lidar_stop, 
        v2v_stop, car_pos)

    def run(self, user_throttle: float, manual_stop: bool,
    lidar_stop: bool, v2v_stop: bool, car_pos: 'np.array') -> float:
        '''
        Stop the car
        :param engine_pwm PWM output by previous components
        :return pwm
        '''
        retval_pwm = min(user_throttle, MAX_THROTTLE)

        if ((self.manual and manual_stop) 
        or (self.lidar and lidar_stop) 
        or (self.v2v and v2v_stop)
        or (self.limit_space and self.out_of_borders(car_pos))):
            retval_pwm = 0.0
            if self.debug:
                print('-- Emergency Brake --- STOP! ', end='')

                if manual_stop:
                    print('Manual')
                elif lidar_stop:
                    print('Lidar')
                elif v2v_stop:
                    print('V2V')
                else:
                    print('Out of Borders')

        if self.debug:
            print('--Emergency Brake -- Throttle:', retval_pwm)
        
        eb_stop = manual_stop or lidar_stop or v2v_stop

        return retval_pwm, eb_stop

    
    def shutdown(self) -> None:
        pass

    def update(self) -> None:
        pass

    def out_of_borders(self, car_pos: 'np.array') -> bool:
        if self.debug:
            print('-- Emergency Brake -- Pos:', car_pos)
        retval = (car_pos[0] < self.borders[0]
                  or car_pos[0] > self.borders[1]
                  or car_pos[2] < self.borders[2]
                  or car_pos[2] > self.borders[3])

        return retval
