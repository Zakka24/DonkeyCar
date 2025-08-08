import os
import time
import termios
import tty
import sys
import logging

from docopt import docopt

logger = logging.getLogger(__name__)

class ControlWithKeyboard:
    '''
    a controller class that outputs the steering and throttle respectively to the input from the keyboard
    '''
        
    def __init__(self, cfg, debug=False):
        self.steering = 0.0
        self.desired_speed = 0.0
        self.current_speed = 0.0
        self.throttle = 0.0
        self.mode = 'user'

        self.running = True
        self.debug = debug
        self.cfg=cfg

        # Constantsa
        self.MIN_SPEED = 0.5
        self.SPEED_VARIATION = 0.1
        self.MIN_THROTTLE = 0.16
        self.THROTTLE_VARIATION = 0.015
        self.MAX_STEERING = 1.0
        self.STEERING_VARIATION = 0.1
        
        # Output flags
        self.stop = False
        self.cc_toggle = False
        self.sc_toggle = False

        self.filedescriptors = None
        self.last_out_stop = False

    def print_to_terminal(self):
        if self.cc_toggle:
            print(f'Speed: {self.desired_speed} m/s')
        else:
            print(f'Throttle: {self.throttle} dc')
            
        
        if not self.sc_toggle:
            print(f'Steering: {self.steering}')
        print('')
            
    def get_input(self):
            self.filedescriptors = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
            key = sys.stdin.read(1)[0]
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.filedescriptors)
            return key
        
    def update(self):
        while self.running:
            input_val = self.get_input()

            self.emergency_brake = False
            
            if input_val == 'w' and not self.cc_toggle:  
                print('> Accelerate')
                if self.throttle == 0.0:
                    self.throttle = self.MIN_THROTTLE
                else:
                    self.throttle += self.THROTTLE_VARIATION
            
            # We will not support going backwards for now
            elif input_val == 's' and not self.cc_toggle:
                print('> Decelerate')
                if self.throttle <= self.MIN_THROTTLE:
                    self.throttle = 0.0
                else:
                    self.throttle = max(self.MIN_THROTTLE,
                        self.throttle - self.THROTTLE_VARIATION)
            
            # Controls Desired Speed (Only with Cruise Control)
            elif input_val == 'l' and self.cc_toggle:
                #Move forward
                print('> Speed up')
                
                if self.desired_speed < self.MIN_SPEED:
                    self.desired_speed = self.MIN_SPEED
                else:
                    self.desired_speed += self.SPEED_VARIATION
            
            elif input_val == 'j' and self.cc_toggle:
                #Slow down
                print('> Slow down')
                
                if self.desired_speed <= self.MIN_SPEED:
                    self.desired_speed = 0.0
                else:
                    self.desired_speed = max(self.MIN_SPEED, 
                        self.desired_speed - self.SPEED_VARIATION)

            elif input_val == ' ':  # TODO: Change with SPACE
                    #Stop
                    print('> Stop!')
                        
                    self.stop = True
                    self.desired_speed = 0.0
                    self.throttle = 0.0

                    if self.debug:
                        self.print_to_terminal()
                
            if input_val == 'd' and not self.sc_toggle:
                #Turn right

                print('> Turn right')
                    
                if (self.steering+self.STEERING_VARIATION <= self.MAX_STEERING):
                    self.steering += self.STEERING_VARIATION
                        
                if self.debug:
                    self.print_to_terminal()
                    
            elif input_val == 'a' and not self.sc_toggle:
                #Turn left
                print('> Turn left')
                    
                if (self.steering-self.STEERING_VARIATION >= -self.MAX_STEERING):
                    self.steering -= self.STEERING_VARIATION
                        
                if self.debug:
                    self.print_to_terminal()
                        
            elif input_val == 'z':
                #Stop
                print('> Straight!')
                    
                self.steering = 0.0
                
                if self.debug:
                    self.print_to_terminal()
            
            elif input_val == 'i':
                # Cruise Control Toggle
                self.cc_toggle = not self.cc_toggle
                
                if self.cc_toggle:
                    self.desired_speed = self.current_speed
                    print('> Cruise Control ON')
                else:
                    self.throttle = 0.0
                    print('> Cruise Control OFF')
                    
            elif input_val == 'o':
                # Steering Control Toggle
                self.sc_toggle = not self.sc_toggle
                
                if not self.sc_toggle:
                    self.steering = 0.0
                    print('> Steering Control OFF')
                else:
                    print('> Steering Control ON')

            if self.debug:
                self.print_to_terminal()
            
            time.sleep(0.01)
    '''
    :param current_speed Is used to initialize desired_speed when
            switch to cruise control mode
    '''
    def run_threaded(self, current_speed: float, mode=None):
        self.current_speed = current_speed
        
        # Block Stop to propagate for more than 1 consecutive iteration
        if self.last_out_stop:
            self.stop = False
        
        self.last_out_stop = self.stop
        
        if self.debug:
            print('-- Keyboard --', end='\t')
            if self.cc_toggle:
                print(f'Speed: {self.desired_speed:.2f}', end = '\t')
            else:
                print(f'Throttle: {self.throttle:.2f}', end = '\t')
            
            if not self.sc_toggle:
                print(f'Steering: {self.steering:.2f}')
            else:
                print()
                

        return (
            self.desired_speed, self.throttle, self.steering,
            self.stop, self.cc_toggle, self.sc_toggle
        )
            
    def shutdown(self):
        time.sleep(.5)
        self.running = False
        logger.info('Stopping Keyboard Controller')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.filedescriptors)
        time.sleep(.5)
