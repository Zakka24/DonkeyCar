#!python

import numpy as np
import json
import random
import math
import time
import socket
import ast
import datetime

from shapely import Point

from donkeycar.parts.path_listener import PathListener
from donkeycar.parts.path_to_be_followed import PathToBeFollowed

import logging

logger = logging.getLogger(__name__)

# TODO: Make keys of dictionaries all integers or all strings

### CONSTANTS ###
CAR_SECURITY_SPACE_RADIUS: float = 0.2  # meters
HARD_STOP_TIME_SPEED: float = 1.3
SOFT_STOP_TIME_SPEED: float = 2.0
CROSS_MARGIN: float = 0.05   # meters
PREC_MARGIN: float = 0.5    # meters
COLLISION_MAX_TIME: float = 4.0 # secs
MAX_COLL_SPEED: float = 1.0 # m/s
SLOW_DIST: float = 4 * CAR_SECURITY_SPACE_RADIUS
MIN_SLOW_SPEED: float = 0.5
SERVER_LISTEN_PERIOD: float = 0.2   # secs
PATH_COLLISION_MAX_STEPS: int = 50 # Steps 
PATH_STEP_PRECISION: float = 0.08  # secs
LINEAR_MOTION_MAX_STEERING: float = 0.2
STEERING_ANGLE_RATIO: float = math.pi / 10  # radians
L_A: float = 0.10
L_R: float = 0.15
LOG_PATH: str = './logs/v2v_ca/{filename}.log'
#################

### MAIN CLASS ###
'''
Vehicle 2 Vehicle Collision Avoidance Component
'''
class V2VCA:
    def __init__(self, cfg, debug: bool = False):
        self.MAX_SPEED = cfg.MAX_SPEED
        self.cfg = cfg

        self.debug: bool = debug
        self.position = np.ndarray(shape=(2,))
        self.rotation: float = 0.0
        self.speed: float = 0.0
        self.steering: float = 0.0
        
        # Stop variables
        self.hard_stop_time: float = 0.0
        self.hard_dist: float = 0.0
        self.hard_stop_pos: np.array = np.array([0.0, 0.0])
        self.soft_stop_time: float = 0.0
        self.soft_dist: float = 0.0
        self.soft_stop_pos: np.array = np.array([0.0, 0.0])
        

        self.v2v_speed: float = self.MAX_SPEED
        self.v2v_hard_stop: bool = False
        self.v2v_softstop: bool = False

        # Stop sender socket
        self.stop_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.prec_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Store here some numpy arrays in order to be more efficient
        self.v1 = np.array([0.0, 0.0])
        self.v2 = np.array([0.0, 0.0])
        self.x2 = np.array([0.0, 0.0])
        
        # 90 degrees right rotation matrix
        self.r_rot = np.array(
            [[0.0, 1.0],
             [-1.0, 0.0]]
        )

        self.run = True
        self.pathListener = PathListener(cfg.PATH_SHARING_PORT, debug = False)
        self.paths = {}     # Store saved paths
        self.precedence = {}
        self.prec_decision: set = set()
        self.add_to_prec_dec: set = set()   # The prec decision check is shited by one cycle

        # Get my Path
        self.path = PathToBeFollowed().get_track()
        
        # For collision time non-linear
        self.my_points = [
            np.array([0.0, 0.0]) for i in range(PATH_COLLISION_MAX_STEPS)]
        self.its_points = [
            np.array([0.0, 0.0]) for i in range(PATH_COLLISION_MAX_STEPS)]
        

        ### Logs
        self.collision = False
        self.path_deadlock = False
        self.prec_deadlock = False
        
        # Format yyyymmdd_HHMMSS
        now = datetime.datetime.now()
        filename = now.strftime("donkey" + str(cfg.DONKEY_ID) 
                                + "_%Y%m%d_%H%M%S")
        self.logfile = open(LOG_PATH.format(filename = filename), 'w')
        
    
    def write_on_log_file(self):
        # Format:
        #   - Timestamp (yyyy-mm-dd HH:MM:SS.ffffff)
        #   - Position  ([+/-xx.xxx, +/-yy.yyy])
        #   - Speed     (vv.vvv)
        #   - Path_Deadlock Prec_Deadlock Collision (y/ny/ny/n)
    
        # Date and Time
        now = datetime.datetime.now()
        ts = now.strftime("%Y-%m-%d %H:%M:%S.%f")
        
        # Flags string
        flags = ('y' if self.collision else 'n') +\
                ('y' if self.path_deadlock else 'n') +\
                ('y' if self.prec_deadlock else 'n')

        # Log message composition
        log_msg = ('{timestamp}\t[{x:2.3f},{y:2.3f}]' +
        '\t{speed:.3f}\t{flags}\n').format(
            timestamp = ts,
            x = self.position[0],
            y = self.position[1],
            speed = self.speed,
            flags = flags
        )
        
        self.logfile.write(log_msg)

    def check_body_msg(self, body: list):
        # Expected content of the body:
        # - Position: list of tow floats
        # - Speed: float
        # - Direction: float
        # - Steer: float

        assert type(body) == list
        assert len(body) == 4
        
        assert type(body[0]) == list
        assert len(body[0]) == 2
        assert type(body[0][0]) == type(body[0][1]) == float
        
        assert type(body[1]) == float
        assert body[1] >= 0

        assert type(body[2]) == float

        assert type(body[3]) == float
        assert -1 <= body[3] and body[3] <= 1

    def send_stop_msg(self, addr: str):
        msg = {
            'id': self.cfg.DONKEY_ID,
            'msg': 'STOP'
        }
        to_send = json.dumps(msg).encode('utf-8')
        
        if self.debug:
            print('-- V2VCA -- Send STOP msg')
        
        self.stop_s.sendto(to_send, (addr, self.cfg.V2V_PORT))
    
    # Send precedence request
    def send_give_prec(self, addr: str):
        msg = {
            'id': self.cfg.DONKEY_ID,
            'msg': 'GIVE PREC'
        }
        
        if self.debug:
            print('-- V2VCA -- Send GIVE PREC msg')
        
        to_send = json.dumps(msg).encode('utf-8')
        self.prec_s.sendto(to_send, (addr, self.cfg.V2V_PORT))

    # Send precedence loan
    def send_take_prec(self, addr: str):
        msg = {
            'id': self.cfg.DONKEY_ID,
            'msg': 'TAKE PREC'
        }
        
        #if self.debug:
        #    print(msg)
        #if self.debug:
        #    print(msg)
        
        to_send = json.dumps(msg).encode('utf-8')
        self.prec_s.sendto(to_send, (addr, self.cfg.V2V_PORT))


    def distance(self, car: list) -> float:
        car_pos = np.array(car[0])
        return np.linalg.norm(self.position - car_pos)
    
    def steering_to_angle(self, steering: float) -> float: 
        # Assume angle is proportional to steering pwm
        return steering * self.cfg.MAX_STEERING_ANGLE
    
    '''
    DEGREES rotation to direction vector
    '''
    def rot_to_vec(self, rot: float) -> np.array:
        return np.array([
            math.cos(math.radians(rot)),
            math.sin(math.radians(rot))
        ])

    def update_stop_times(self):
        self.hard_stop_time = HARD_STOP_TIME_SPEED * self.speed
        self.soft_stop_time = SOFT_STOP_TIME_SPEED * self.speed


    def update_stop_positions(self):
        
        if self.speed == 0.0:
            self.hard_dist = 0.0
            self.soft_dist = 0.0
            self.hard_stop_pos = np.copy(self.position)
            self.soft_stop_pos = np.copy(self.position)
            return

        # Use constant deceleration
        self.hard_dist = self.speed * self.hard_stop_time\
            - (self.hard_stop_time**2 / HARD_STOP_TIME_SPEED)/2
        
        self.soft_dist = self.speed * self.soft_stop_time\
            - (self.soft_stop_time**2 / SOFT_STOP_TIME_SPEED)/2

        if self.cfg.PATH_CA and self.sc_toggle:
            p = Point(self.position[0], self.position[1])
            dist_0 = self.path.boundary.project(p)
            
            # Hard Stop
            hard_dist_stop = dist_0 - self.hard_dist
            hard_stop_point = self.path.boundary.interpolate(
                hard_dist_stop)   
            self.hard_stop_pos[0] = hard_stop_point.x
            self.hard_stop_pos[1] = hard_stop_point.y
            
            # Soft Stop
            soft_dist_stop = dist_0 - self.soft_dist
            soft_stop_point = self.path.boundary.interpolate(
                soft_dist_stop)
            self.soft_stop_pos[0] = soft_stop_point.x
            self.soft_stop_pos[1] = soft_stop_point.y


        elif (self.cfg.CIRC_CA and 
        abs(self.steering) > LINEAR_MOTION_MAX_STEERING):
            theta, C, r_r, r_m, w = self.get_circular_motion_vars(
                self.position, self.speed, self.rotation, self.steering
            )   
            
            #if self.debug:
            #    print('-- V2VCA -- Angular Velocity:', w)

            R = r_m

            # Hard Stop
            theta_t_hard = theta + (self.hard_dist / R)
            self.hard_stop_pos = (C 
                + (r_r * np.array(
                    [math.sin(theta_t_hard), 
                    -math.cos(theta_t_hard)]
                ))
                + (L_R * np.array(
                    [math.cos(theta_t_hard), 
                    math.sin(theta_t_hard)]
                ))
            )
            
            # Soft Stop
            theta_t_soft = theta + (self.soft_dist / R)
            self.soft_stop_pos = (C 
                + (r_r * np.array(
                    [math.sin(theta_t_soft), 
                    -math.cos(theta_t_soft)]
                ))
                + (L_R * np.array(
                    [math.cos(theta_t_soft), 
                    math.sin(theta_t_soft)]
                ))
            )


        else: # Assume linear stop motion
            d = np.array([
                math.cos(math.radians(self.rotation)), 
                math.sin(math.radians(self.rotation))
            ])
           
            self.hard_stop_pos = self.position +\
                self.hard_dist * d
            self.soft_stop_pos = self.position +\
                self.soft_dist * d
    
    '''
    Calculate the distance between car_pos and the linear trajectory
    of the car that here we call `path`
    '''
    def distance_from_linear_motion(self, car_pos: np.array, 
        path_pos: np.array, path_rot: float):
        
        path_rot_rad = math.radians(path_rot)
        
        # Direction versor
        u = np.array([
            math.cos(path_rot_rad),
            math.sin(path_rot_rad)])
        
        # Motion (almost) parallel to one dimension
        if abs(u[0]) < 0.05:
            return abs(path_pos[0] - car_pos[0])
        elif abs(u[1]) < 0.05:
            return abs(path_pos[1] - car_pos[1])
    
        w = np.array([1.0 / u[0], -1.0 / u[1]])
        b = (path_pos[1] / u[1]) - (path_pos[0] / u[0])

        c = np.inner(w, car_pos) + b

        return abs(c / np.linalg.norm(w))


    def distance_from_circular_motion(self, car_pos: np.array,
        path_pos: np.array, path_rot: float, path_steer: float):
        # Find distance from the center and
        # subtract the radius of the trajectory.
        # Take the absolute value
        
        # TODO: Substitute with get_circular_motion_vars

        # Car angle and Steering angle (in radians)
        theta = math.radians(path_rot)
        alpha = path_steer * STEERING_ANGLE_RATIO
        
        # Rear Length Vector
        l_r_v = L_R * np.array([math.cos(theta), math.sin(theta)])
        # Radius
        r_r = (L_A + L_R) / math.tan(alpha)
        # Circle radius vector
        r_r_v = np.array(
            [r_r * math.cos(theta), 
             r_r * math.sin(theta)]
        )

        # Center of circle
        C = np.array(path_pos) - l_r_v - r_r_v
        
        # Middle radius
        r_m = math.sqrt(L_R ** 2 + r_r ** 2)
        
        ### Calculations
        dist_from_C = np.linalg.norm(car_pos - C)
        
        return abs(dist_from_C - r_m)


    def distance_from_path(self, car_pos: np.array, path_id: int):
        # It should work I guess
        path = None
        if path_id == self.cfg.DONKEY_ID:
            path = self.path
        else:
            path = self.paths[path_id]

        point = Point(car_pos[0], car_pos[1])
        return abs(path.exterior.distance(point))
    
    def get_circular_motion_vars(self, pos: list, speed: float, 
    rot: float, steer: float) -> tuple:
        
        ### Calculate useful variables
        # Car angle and Steering angle (in radians)
        theta = math.radians(rot)
        alpha = -steer * STEERING_ANGLE_RATIO
        
        # Rear Length Vector
        l_r_v = L_R * np.array([math.cos(theta), math.sin(theta)])
        # Radius
        # It has a sign: positive if counterclockwise,
        #                negative otherwise
        r_r = (L_A + L_R) / math.tan(alpha)
        r_m = math.copysign(math.sqrt(r_r**2 + L_R**2), r_r)

        # Circle rear radius vector
        r_r_v = math.copysign(r_r, alpha) * np.array(
            [math.sin(theta), 
             -math.cos(theta)]
        )
        
        # Center of circle
        C = np.array(pos) - l_r_v - r_r_v
        
        # Angular velocity
        w = speed / r_m
        
        #if self.debug:
        #    print('-- V2VCA -- Angular Velocity:', w_v)
        #    print('-- V2VCA -- Center:', C)
        #    print('-- V2VCA -- Rear Radius:', r_r)
        #    print('-- V2VCA -- Steering Angle:', alpha)
        #    print()

        return theta, C, r_r, r_m, w


    def quantize_linear_motion(self, car_id: int, car: list) -> list:
        # Use self.x2 and self.v2 to store vectorized speed
        self.x2[0] = car[0][0]
        self.x2[1] = car[0][1]
        
        car_rot_rad = math.radians(car[2])
        self.v2[0] = car[1] * math.cos(car_rot_rad)
        self.v2[1] = car[1] * math.sin(car_rot_rad)
        
        # res becomes a reference to one of the class attriutes
        res = None
        if car_id == self.cfg.DONKEY_ID:
            res = self.my_points
        else:
            res = self.its_points
        
        # Assign values
        for i in range(PATH_COLLISION_MAX_STEPS):
            t = i * PATH_STEP_PRECISION
            pos_t = self.x2 + (self.v2 * t)
            res[i][0] = pos_t[0]
            res[i][1] = pos_t[1]
        
        return res


    def quantize_circular_motion(self, car_id: int, car: list) -> list:
        
        theta, C, r_r, r_m, w = self.get_circular_motion_vars(
            car[0], car[1], car[2], car[3]
        )

        ### Get array to use
        # res becomes a reference to one of the class attriutes
        res = None
        if car_id == self.cfg.DONKEY_ID:
            res = self.my_points
        else:
            res = self.its_points
    
        ### Fill res
        for i in range(PATH_COLLISION_MAX_STEPS):
            t = i * PATH_STEP_PRECISION
            theta_t = theta + (w * t)
            
            res[i] = (C
            + (r_r * np.array([math.sin(theta_t), -math.cos(theta_t)]))
            + (L_R * np.array([math.cos(theta_t), math.sin(theta_t)])))
        
        return res


    def quantize_path(self, car_id: int, car: list) -> list:
        # Just use interpolate function
        path_boundary = None
        res = None
        if car_id == self.cfg.DONKEY_ID:
            path_boundary = self.path.boundary
            # res becomes a reference to one of the class attriutes
            res = self.my_points
        
        else:
            path_boundary = self.paths[car_id].boundary
            # res becomes a reference to one of the class attriutes
            res = self.its_points
        
        p = Point(car[0][0], car[0][1])
        dist_0 = path_boundary.project(p)
        
        # Assign values
        for i in range(PATH_COLLISION_MAX_STEPS):
            t = i * PATH_STEP_PRECISION
            dist_t = dist_0 - (car[1] * t)
            point_t = path_boundary.interpolate(dist_t)
            res[i][0] = point_t.x
            res[i][1] = point_t.y

        return res

    '''
    Directly solve the inequality
    '''
    def collision_time_linear_motion(self, car: list) -> tuple:
        # Init references to make code more readable
        v1 = self.v1
        v2 = self.v2
        x1 = np.copy(self.position)
        x2 = self.x2

        # Define speeds and positions
        car_rot_rad = math.radians(car[2])
        v2[0] = car[1] * math.cos(car_rot_rad)
        v2[1] = car[1] * math.sin(car_rot_rad)

        x2[0] = car[0][0]
        x2[1] = car[0][1]

        # Coefficients of quadratic equation
        R = CAR_SECURITY_SPACE_RADIUS
        A = (v1[0] - v2[0])**2 + (v1[1] - v2[1])**2
        B = 2 * (
            (x1[0] - x2[0]) * (v1[0] - v2[0]) +
            (x1[1] - x2[1]) * (v1[1] - v2[1])
        )
        C = (x1[0] - x2[0])**2 + (x1[1] - x2[1])**2 - (4 * R**2)

        # Solve equation
        Delta = B**2 - 4*A*C
        
        if A > 0 and Delta >= 0:
            t_1 = (-B - math.sqrt(Delta)) / (2*A)
            t_2 = (-B + math.sqrt(Delta)) / (2*A)
        else:
            t_1 = t_2 = -1.0
        
        return t_1, t_2
    
    '''
    Cannot directly solve this; need to make frames of the future
    '''
    def get_collision_nonlinear_motion(self, car_id: int, car: list):
        # Get right quantised trajectory
        # List informations of this car
        myid = self.cfg.DONKEY_ID
        mycar = [
            [self.position[0], self.position[1]],
            self.speed,
            self.rotation,
            self.steering
        ]

        # My points
        my_points = None
        if self.cfg.PATH_CA and self.sc_toggle:
            my_points = self.quantize_path(myid, mycar)
        elif (self.cfg.CIRC_CA and 
        abs(self.steering) > LINEAR_MOTION_MAX_STEERING):
            my_points = self.quantize_circular_motion(myid, mycar)
            #print('Circ')
        else:
            my_points = self.quantize_linear_motion(myid, mycar)
            #print('Line')

        #if self.debug:
        #    print(my_points)
        
        # Its points
        if self.cfg.PATH_CA and car_id in self.paths:
            its_points = self.quantize_path(car_id, car)
        elif (self.cfg.CIRC_CA and 
        abs(car[3]) > LINEAR_MOTION_MAX_STEERING):
            its_points = self.quantize_circular_motion(car_id, car)
        else:
            its_points = self.quantize_linear_motion(car_id, car)
        
        # Iterate over discrete time
        min_dist = float('inf')
        for i in range(1, PATH_COLLISION_MAX_STEPS):
            # Find the minimum t within the interval where the
            # two cars get too close. If they don't, return -1.0
            dist = np.linalg.norm(my_points[i] - its_points[i])
            min_dist = min(min_dist, dist)
            if dist <= 2 * CAR_SECURITY_SPACE_RADIUS:
                #if self.debug:
                #    print('-- V2VCA -- Dist:', dist)
                
                t = i * PATH_STEP_PRECISION
                
                if self.speed > 0.1:
                    my_pos = my_points[i]
                    my_ds = my_points[i] - my_points[i-1]
                    my_rot = math.degrees(np.arctan2(my_ds[1], my_ds[0]))
                else:
                    my_pos = np.copy(self.position)
                    my_rot = self.rotation
                
                if car[1] > 0.1:
                    its_pos = its_points[i]
                    its_ds = its_points[i] - its_points[i-1]
                    its_rot = math.degrees(np.arctan2(its_ds[1], its_ds[0]))
                else:
                    its_pos = np.array(car[0])
                    its_rot = car[2]

                return t, my_pos, my_rot, its_pos, its_rot, min_dist

        #if self.debug:
        #    print('-- V2VCA -- Min dist:', min_dist)
        #    print('-- V2VCA -- Max lookahead:', 
        #          self.speed * PATH_COLLISION_MAX_STEPS * PATH_STEP_PRECISION)
        #
        #    print(my_points)
        #    print(its_points)

        return (-1.0, np.copy(self.position), self.rotation, 
                np.array(car[0]), car[2], min_dist)


    def get_collision(self, car_id: int, car: list) -> np.array:
        
        if (
            (not self.cfg.PATH_CA or 
            (not self.sc_toggle and not car_id in self.paths))
            and
            (not self.cfg.CIRC_CA or 
            ((abs(car[3]) <= LINEAR_MOTION_MAX_STEERING)
             and abs(self.steering) <= LINEAR_MOTION_MAX_STEERING))
        ):
            
            t_1, t_2 = self.collision_time_linear_motion(car)
            
            my_pos = np.copy(self.position)
            my_rot = self.rotation
            its_pos = np.array(car[0])
            its_rot = car[2]
            min_dist = float('inf')

            if t_2 < 0:     # Linear motion never collides
                t_1 = -1.0
            elif t_1 <= 0:  # Linear motion is colliding
                t_1 = 0.0
            else:

                # Variables to return
                my_rot = self.rotation
                my_rot_v = np.array([
                    math.cos(math.radians(my_rot)),
                    math.sin(math.radians(my_rot))
                ])
                my_pos = self.position + (my_rot_v * t_1)        
                
                its_rot = car[2]
                its_rot_v = np.array([
                    math.cos(math.radians(its_rot)),
                    math.sin(math.radians(its_rot))
                ])
                its_pos = np.array(car[0]) + (its_rot_v * t_1)

            return t_1, my_pos, my_rot, its_pos, its_rot, min_dist
            
        else:
            return self.get_collision_nonlinear_motion(car_id, car)
    

    '''
    Car 2 is in front of Car 1
    '''
    def is_in_front(self, car1_pos: np.array, car1_dir: np.array,
        car2_pos: np.array):
        
        d = car2_pos - car1_pos
        i = np.inner(d, car1_dir)
        return i > 0.0
    
    '''
    Car 2 is on the right of Car 1
    '''
    def is_on_the_right(self, car1_pos: np.array, car1_dir: np.array,
        car2_pos: np.array):
        
        r_vec = self.r_rot.dot(car1_dir)
        return self.is_in_front(car1_pos, r_vec, car2_pos)


    '''
    Find the distance between car_pos and the trajectory of traj_car
    '''
    def get_distance_from_trajectory(self, car_pos: np.array,
        traj_car_id: int, traj_car: list):
       
        dist_from_traj = 0.0

        if self.cfg.PATH_CA and (int(traj_car_id) in self.paths or
        (int(traj_car_id) == self.cfg.DONKEY_ID and self.sc_toggle)):

            # Project on path
            dist_from_traj = (
                self.distance_from_path(
                    car_pos, int(traj_car_id)
                )
            )
        elif self.cfg.CIRC_CA and abs(traj_car[3]) > LINEAR_MOTION_MAX_STEERING:
            # Project on circular motion
            dist_from_path = (
                self.distance_from_circular_motion(
                    car_pos, np.array(traj_car[0]),
                    traj_car[2], traj_car[3]
                )
            )
        else:
            # Project on linear motion
            dist_from_traj = (
                self.distance_from_linear_motion(
                    car_pos, np.array(traj_car[0]),
                    traj_car[2]
                )
            )
        
        return dist_from_traj

    
    def compute_precedence(self, car_id: int, car_speed: float, 
    my_pos: np.array, my_rot: float, its_pos: np.array, 
    its_rot: float) -> bool:
        
        my_rot_vec = self.rot_to_vec(my_rot)
        its_rot_vec = self.rot_to_vec(its_rot)
        
        # Collision position check
        it_is_in_front = self.is_in_front(my_pos, my_rot_vec, its_pos)
        i_am_in_front = self.is_in_front(its_pos, its_rot_vec, my_pos)
        
        if it_is_in_front and not i_am_in_front:
            print('-- V2VCA -- Front')
            return True
        elif not it_is_in_front and i_am_in_front:
            return False
        
        # Right check
        it_is_on_my_right = self.is_on_the_right(my_pos, my_rot_vec, its_pos)
        i_am_on_its_right = self.is_on_the_right(its_pos, its_rot_vec, my_pos)

        res = it_is_on_my_right and not i_am_on_its_right

        if res and self.debug:
            print('-- V2VCA -- I am on the left')

        if it_is_on_my_right == i_am_on_its_right:
            # Speed check
            res = self.speed > car_speed + 0.15  # Admit an error

            if res and self.debug:
                print('-- V2VCA -- I am faster')

            if not res and not car_speed > self.speed + 0.15: # Admit an error
                # ID check
                res = self.cfg.DONKEY_ID > int(car_id)
                if res and self.debug:
                    print('-- V2VCA -- I have a bigger ID')
        
        return res


    def hard_stop(self) -> None:
        # STOP!
        self.v2v_speed = 0.0
        self.v2v_hard_stop = True
    
    def soft_stop(self) -> None:
        self.v2v_speed = 0.0
        self.v2v_soft_stop = True

    def wait(self, car_id: int, car: list) -> None:
        # WAIT for donkey_id to pass
        self.precedence[car_id] = min(self.distance(car), 
            8*CAR_SECURITY_SPACE_RADIUS)
        
        if abs(self.speed) < 0.05:  # If almost still
            self.soft_stop()
        else:
            self.hard_stop()


    # Listen
    def update(self):
        while self.run:
            car_id, car_path = self.pathListener.listen()
            self.paths[car_id] = car_path
            time.sleep(SERVER_LISTEN_PERIOD)

    def run_threaded(self, position: np.ndarray, rotation: float, 
        speed: float, steering: float, close_cars_str: str, 
        close_addrs_str: str, to_remove: np.array, sc_toggle: bool, 
        stop_msg: bool, prec_requests: str, prec_loans: str):
        
        #if self.debug:
        #    print('-- V2VCA --', position, rotation, speed, steering, 
        #        close_cars_str, close_addrs_str, to_remove, sc_toggle,
        #        stop_msg)

        min_v2v_speed = self.MAX_SPEED
        self.v2v_hard_stop = False   # Or the car will be stopped forever
        self.v2v_soft_stop = False
        self.collision = False
        self.path_deadlock = False
        self.prec_deadlock = False
        
        # Remove unused paths
        for k in to_remove:
            self.paths.pop(int(k), None)
            self.precedence.pop(k, None)
            self.prec_decision.discard(k)

            #if self.debug:
            #    print('-- V2VCA -- To remove!')


        ### Read the remaining
        try:

            assert type(position) == np.ndarray
            assert len(position) == 3
            #assert type(rotation) == float
            assert type(speed) == float
            assert speed >= 0.0
            assert type(sc_toggle) == bool
    
            self.position[0] = position[0]
            self.position[1] = position[2]
            self.rotation = float(rotation)
            self.speed = speed
            self.steering = steering
            self.sc_toggle = sc_toggle

            # Vectorial Speeds
            self_rot_rad = math.radians(self.rotation)
            self.v1[0] = self.speed * math.cos(self_rot_rad)
            self.v1[1] = self.speed * math.sin(self_rot_rad)

            # Updates
            self.update_stop_times()
            self.update_stop_positions()
            
            # Unpack close cars
            assert type(close_cars_str) == str
            close_cars = json.loads(close_cars_str)
            assert type(close_cars) == dict
            
            # Unpack close addrs
            assert type(close_addrs_str) == str
            close_addrs = json.loads(close_addrs_str)
            assert type(close_addrs) == dict
            
            for k in list(close_cars.keys()):
                self.check_body_msg(close_cars[k])

        except Exception as e:
            if self.debug:
                print('-- V2VCA --', e)
            
        # GIVE PREC messages
        prec_reqs = set(ast.literal_eval(prec_requests))
        
        # TAKE PREC messages
        for k in ast.literal_eval(prec_loans):
            if k in ast.literal_eval(prec_requests):
                continue

            if k in self.precedence:    # Possible Deadlock
                if self.debug:
                    print('-- V2VCA -- Prec Deadlock!')
                self.prec_deadlock = True
                self.add_to_prec_dec.add(k)
                self.precedence.pop(k, None)

        
        ### Check in Precedence
        # Stop if not empty
        if stop_msg or self.precedence or self.prec_decision:
            if abs(self.speed) < 0.05:  # If almost still
                self.soft_stop()
            else:
                self.hard_stop()

        
        ### Check Close cars
        for k in list(close_cars.keys()):
            car = close_cars[k]
            
            dist = self.distance(car)

            ### HARD Stop Conditions
            if dist <= 2 * CAR_SECURITY_SPACE_RADIUS:
                if self.debug:
                    print('-- V2VCA -- Close!')
                
                self.collision = True
                self.hard_stop()
                self.send_stop_msg(close_addrs[k])
                
                self.precedence.pop(k, None)
                self.prec_decision.discard(k)

            # Get Expected Collision Time
            (collision_time, my_col_pos, my_col_rot,
             its_col_pos, its_col_rot, min_dist) = self.get_collision(k, car)
            
            #if self.debug:
            #    print('-- V2VCA -- My Collision Position:', my_col_pos)
            #    print('-- V2VCA -- My Collision Rotation:', my_col_rot)
            #    print('-- V2VCA -- Its Collision Position:', its_col_pos)
            #    print('-- V2VCA -- Its Collision Rotation:', its_col_rot)

            # Check GIVE PREC messages
            if (k in prec_reqs and not k in self.precedence
            and self.hard_dist >= collision_time * self.speed
            - PREC_MARGIN):
                self.wait(k, close_cars[k])
                self.send_take_prec(close_addrs[k])
            
            #if self.debug:
            #    print('-- V2VCA -- Collision time:', collision_time)

            if (collision_time > 0 and collision_time * self.speed 
            < self.hard_dist):
                if self.is_in_front(my_col_pos, 
                self.rot_to_vec(my_col_rot), its_col_pos):
                    #print('Stop dist:', self.hard_dist)
                    if self.debug:
                        print('-- V2VCA -- Stop!')
                    
                    self.hard_stop()
                    self.add_to_prec_dec.add(k)
                    self.precedence.pop(k, None)

                else:
                    self.send_stop_msg(close_addrs[k])
            
            #if self.debug:
            #    print('-- V2VCA -- On my right:', self.is_on_the_right(car))
            #    print('-- V2VCA -- On its right:', self.i_am_on_its_right(car))
            #    print('-- V2VCA -- I am above:', self.i_am_above(car))
            #    print('-- V2VCA -- Collision time:', collision_time)
            
            #if self.debug:
            #    print('-- V2VCA -- Distance:', self.distance(close_cars[k]))
            ### Check in Precedence
            if k in self.precedence:
                # Send TAKE PREC message
                self.send_take_prec(close_addrs[k])
                
                # Goto Run
                if (self.distance(close_cars[k]) > self.precedence[k] + 0.1):
                    if self.debug:
                        print('-- V2VCA -- Remove', k, 'from precedences')
                    self.precedence.pop(k, None)

                # Last time precedence decision process
                elif self.is_in_front(np.array(car[0]), 
                self.rot_to_vec(car[2]), self.position): # I am in front
                    my_dft = self.get_distance_from_trajectory(
                        self.position, 
                        k, 
                        close_cars[k]
                    )
                    
                    its_dft = self.get_distance_from_trajectory(
                        np.array(close_cars[k][0]),
                        self.cfg.DONKEY_ID,
                        [
                            self.position, 
                            self.speed, 
                            self.rotation,
                            self.steering
                        ]
                    )
                    
                    if (self.is_in_front(self.position, 
                        self.rot_to_vec(self.rotation), np.array(car[0])) # I is in front of me
                    and my_dft <= 2*CAR_SECURITY_SPACE_RADIUS + CROSS_MARGIN
                    and its_dft <= 2*CAR_SECURITY_SPACE_RADIUS + 
                    CROSS_MARGIN):
                        if self.debug:
                            print('-- V2VCA -- PATH DEADLOCK')
                        self.send_stop_msg(close_addrs[k])
                        self.path_deadlock = True

                    elif (my_dft <= 2*CAR_SECURITY_SPACE_RADIUS + 
                    CROSS_MARGIN):
                        if self.debug:
                            print('-- V2VCA -- Change! Take Precedence from', 
                                k)
                        
                        self.add_to_prec_dec.add(k)
                        self.precedence.pop(k, None)
                        self.send_give_prec(close_addrs[k])

                    # else keep giving precedence
                # else keep giving precedence

            ### Not in precedence
            elif ((k in self.prec_decision or (collision_time > 0 
            and collision_time <= COLLISION_MAX_TIME))
            and self.compute_precedence(k, car[1], self.position, 
            self.rotation, np.array(car[0]), car[2])):               
                if self.debug:
                    print('-- V2VCA -- I\'ll calculate precedence')
                

                # Maximum min speed
                min_v2v_speed = min(min_v2v_speed, MAX_COLL_SPEED)

                hard_dft = self.get_distance_from_trajectory(
                    self.hard_stop_pos, k, car)
                
                #if self.debug:
                #    print('-- V2VCA -- Hard Distance:', self.hard_dist)
                #    print('-- V2VCA -- Collision Distance:', 
                #          collision_time * self.speed)
                
                # there will be a collision
                i_am_in_front = self.is_in_front(np.array(car[0]), 
                    self.rot_to_vec(car[2]), self.position)

                # Precedence decision
                # TODO: Check if correct
                if (k in self.prec_decision or 
                self.hard_dist >= collision_time * self.speed
                - PREC_MARGIN - 2 * CAR_SECURITY_SPACE_RADIUS):
                    if ((hard_dft <= CROSS_MARGIN + 
                    (2 * CAR_SECURITY_SPACE_RADIUS) and i_am_in_front) 
                    or 
                    (not (self.position == my_col_pos).all()
                    and not self.is_in_front(self.position,
                    self.rot_to_vec(self.rotation), my_col_pos))):
                        if self.debug:
                            print('-- V2VCA -- Take Precedence from', k)
                            print('-- V2VCA --', self.hard_stop_pos, hard_dft)
                            
                        # Take precedence
                        self.send_give_prec(close_addrs[k])
                    
                    else:
                        if self.debug:
                                print('-- V2VCA -- Give Precedence to', k)
                                #print('-- V2VCA -- Hard DFT:', hard_dft)
                                #print('-- V2VCA -- Hard Stop Time:', 
                                #      self.hard_stop_time)
                                #print('-- V2VCA -- Hard Stop Pos:', 
                                #      self.hard_stop_pos)
                        # Give precedence
                        self.wait(k, car)
                        self.send_take_prec(close_addrs[k])
                    
                # Soft Stop
                elif (self.soft_dist >= collision_time * self.speed):
                    if self.debug:
                        print('-- V2VCA -- Soft Stop')
                    self.soft_stop()    # Slow down to 0.0

                # Slow Down if car on the right
                elif self.cfg.AUTO_SLOW:
                    if self.debug:
                        print('-- V2VCA -- Slow Down!')
                    min_v2v_speed = min(min_v2v_speed, 
                        max(self.speed / 4, MIN_SLOW_SPEED))
            
            elif (min_dist <= SLOW_DIST
            and self.compute_precedence(k, car[1], self.position, 
                self.rotation, np.array(car[0]), car[2])
            and self.cfg.AUTO_SLOW):
                if self.debug:
                    print('-- V2VCA -- Slow Down!')
                min_v2v_speed = min(min_v2v_speed, 
                    max(self.speed / 2, MIN_SLOW_SPEED))
                
        
        # Reset precedence decision for the next cycle
        self.prec_decision = self.add_to_prec_dec.copy()
        self.add_to_prec_dec.clear()

        if not (self.v2v_hard_stop or self.v2v_soft_stop):
            self.v2v_speed = min_v2v_speed
        
        #if self.debug:
        #    print(self.v2v_speed) # Newline
        #    print('Precedences:', self.precedence)
        #    print(self.paths)   

        self.write_on_log_file()

        return self.v2v_speed, self.v2v_hard_stop
    

    def shutdown(self):
        time.sleep(.5)
        self.run = False
        self.pathListener.shutdown()
        self.stop_s.close()
        self.prec_s.close()
        self.logfile.close()
        time.sleep(.5)
