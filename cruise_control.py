#!python3

import time
import scipy
import numpy as np

### CONSTANTS ###
dt: float = 0.1 # SI call period
DESIRED_SPEED_STEP: float = 0.05 # m/s^2

### MAIN CLASS ###

class CruiseControl:
    def __init__(self, cfg, debug = False) -> None:
        # Init variables
        self.debug = debug
        
        if cfg.DRIVE_LOOP_HZ:
            self.desired_speed_step = DESIRED_SPEED_STEP / cfg.DRIVE_LOOP_HZ 
        else:
            self.desired_speed_step = DESIRED_SPEED_STEP / 20 


        # Speeds
        self.current_speed: float = 0.0
        self.desired_speed: float = 0.0
        self.last_speed: float = 0.0

        # Used inside Proportional Integral Controller
        self.last_error: float = 0.0
        self.adjusted_acceleration: float = 0.0
        self.adjusted_throttle: float = 0.0
        self.last_throttle: float = 0.0
        self.user_throttle: float = 0.0

        self.overshoot: float = 0.0 # Maximum error
        self.already_calculed: bool = False # ?

        # SI Parameters
        self.kp: float = 0.2
        self.ki: float = 0.2
        self.gamma: float = 0.0973

        # Times init
        # Time between each Proportional Integral Controller call
        self.start_time: float = time.time()    
        self.time_to_hit: float = time.time()   # ?
        
        self.cfg = cfg
        self.running: bool = True


    def stop(self):
        self.last_speed = 0.0
        self.last_throttle = 0.0
        self.last_error = 0.0
        self.desired_speed = 0.0
    
    def reset(self):
        self.last_speed = self.current_speed - 0.01
        self.desired_speed = self.current_speed
        self.last_error = 0.0

    def get_user_throttle(self) -> float:
        '''
        Properly set self.adjusted_throttle for sending out
        :return: user_throttle
        '''

        user_throttle = self.adjusted_throttle
        
        #if self.debug:
        #    print('-- Cruise Control -- User Throttle:', user_throttle)
        
        # skipping the engine dead zone, selected throttle zone 
        # of +-0.01 because we cannot add (or subtract) 0.115 
        # when the car needs to brake (adjusted_throttle = 0)
        if(self.desired_speed > 0.0):
            if self.current_speed < self.cfg.START_SPEED:
                user_throttle = user_throttle +\
                self.cfg.START_THROTTLE * np.sign(user_throttle) 
                self.last_error = max(0.0, self.last_error)
            else:
                user_throttle = user_throttle +\
                self.cfg.MIN_THROTTLE * np.sign(user_throttle) 


        # avoids the ingagement of the reverse
        user_throttle = max(0.0, user_throttle)

        #if self.debug:
        #    print('-- Cruise Control -- User Throttle:', user_throttle)
        return user_throttle
    
    def speed_to_throttle(self, end_time: float) -> None:
        '''
        Set throttle needed to reach a given speed
        Modify self.last_throttle
        :param end_time: calculated by the calling function
        '''
        
        # Current speed at end_time
        speed = self.last_speed +\
            self.adjusted_acceleration * (end_time - self.start_time)
        

        self.last_speed = speed

        self.adjusted_throttle = self.gamma * speed
        
        if self.adjusted_throttle > 1.0:
            self.adjusted_throttle = 1.0
        
        elif self.adjusted_throttle < -1.0:
            self.adjusted_throttle = -1.0
        
        self.last_throttle = self.adjusted_throttle
        
        #if self.debug:
        #    print('-- Cruise Control -- Speed:', speed)
        
    
    def proportional_integral_controller(self) -> None:
        '''
        Compute PI and write class attributes
        '''
            
        end_time = time.time()  # Used in Integral
        
        current_error = self.current_speed - self.desired_speed
        
        if self.debug:
            print('-- Cruise Control -- Current Error:', current_error)
        
        # Calculate the area of the trapezoid delimited by
        # the points:
        #   - (self.start_time, 0)
        #   - (end_time, 0)
        #   - (end_time, current_error)
        #   - (self.start_time, self.last_error)
        integral = scipy.integrate.trapezoid(
            [self.last_error, current_error], 
            [self.start_time, end_time]
        )

        used_kp = self.kp * current_error
        used_ki = self.ki * integral
        
        self.adjusted_acceleration = - used_kp - used_ki
        self.last_error = current_error
        
        #if self.debug:
        #    print('-- Cruise Control -- Adjusted Acceleration:', self.adjusted_acceleration)
        
        self.speed_to_throttle(end_time)
        self.start_time = end_time
        
        # TODO: Some logs
        
        # Update overshoot
        self.overshoot = max(self.overshoot, current_error)
        
        # TODO: To understand
        if self.current_speed >= self.desired_speed and\
        self.desired_speed > 0 and not(self.already_calculed):
            self.time_to_hit = time.time() - self.time_to_hit
            self.aready_calculed = True

    
    def run_threaded(self, current_speed: float, desired_speed: float,
        user_throttle: float, cc_toggle: bool, eb_stop: bool) -> tuple:
        '''
        Compute throttle given desired and current speeds
        :param current_speed (current/speed): 
            given by encoder or speed_computator
        :param desired_speed (desired/speed): 
            given by a controller
        :return (user/speed):
            the expected speed by the cruise control
        :return (user/throttle): 
            the calculated throttle
        '''
        #print('CC:', desired_speed)
        self.user_throttle = user_throttle
        try:
            # Validity Check
            assert current_speed != None
            assert desired_speed != None
            assert type(current_speed) == float
            assert type(desired_speed) == float
            assert 0 <= current_speed # Does not work with negative speeds
            assert 0 <= desired_speed # Does not work with negative speeds
            
            # Update speed values
            self.current_speed = round(current_speed, 3)
            #if desired_speed > self.desired_speed:
            #    self.desired_speed = round(min(
            #        desired_speed,
            #        self.desired_speed + self.desired_speed_step
            #    ), 3)
            #elif desired_speed < self.desired_speed:
            #    self.desired_speed = round(max(
            #        desired_speed,
            #        self.desired_speed - self.desired_speed_step
            #    ), 3)
            self.desired_speed = desired_speed
                
        except Exception as e:
            if self.debug:
                print('An error occurred in Cruise Control.\n', type(e))
        
        if not cc_toggle:
            self.stop()

        if eb_stop:
            self.stop()
        
        return self.get_user_throttle()
    
    #def run(self, current_speed: float, desired_speed: float) -> tuple:
    #    '''
    #    Should not be used
    #    '''
    #    return (self.last_speed, self.get_user_throttle())
    
    def shutdown(self) -> None:
        self.running = False

    def update(self) -> None:
        while self.running:
            time.sleep(dt)
            # If changes, calculate PI
            #if not self.current_speed == self.desired_speed:
            # Call for PI
            self.proportional_integral_controller()

