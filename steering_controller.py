#!python3
 
import json
import time
import sys
import numpy as np
import quaternion as quaternion
from donkeycar.parts.path_to_be_followed import PathToBeFollowed

from shapely.geometry import Point
from scipy.spatial.transform import Rotation
import scipy

import matplotlib.pyplot as plt

import logging
logger = logging.getLogger(__name__)
from datetime import datetime

import socket
import json

### CONSTANTS ###
#attributes expected in json message
POS_KEY: str = 'position'
ROT_KEY: str = 'rotation'

NEXT_DISTANCE: float = -0.5 #how much the new next point is distant
CORRECTION = False
CORRECTION_STEER_SIGN = False

# distance_error
P_DISTANCE = True
kp_distance : float = 8 #8 #2.3
I_DISTANCE = False
ki_distance : float = 0.5
D_DISTANCE = False
kd_distance : float = 0.003

# angle_error
P_ANGLE = True
kp_angle : float = 0.05 #0.05 #0.02
I_ANGLE = False
ki_angle : float = 0.005
D_ANGLE = False
kd_angle : float = -0.00003

class SteeringController:
    '''
    This part will control the steering.
    Given actual position and the track, the part calculates 
    how much the car need to steer in order to reach the desired position.
    A log file will be written as long as the part is running.
    '''

    def __init__(self, cfg, debug=False, log_file="steer", show_graph=False):

        self.mode = 'user'
        self.debug = debug
        # At start, we don't know where the car is, we need to wait for the optitrack msg
        self.position = Point(0.0, 0.0)                 #X, Z
        
        self.track = PathToBeFollowed().get_track()
        self.steer = 0.0                              #value between [-1, 1]
        self.car_angle = 0.0
        self.args = None

        self.last_error = 0.0
        self.distance_error = 0.0
        self.angle_error = 0.0
        self.control = None

        self.closest_point = Point(0.0,0.0)
        self.next_closest_point = Point(0.0,0.0)
        self.track_angle = 0.0
        
        self.ret_closest_point = np.ndarray(shape = (2,))
        self.ret_next_closest_point = np.ndarray(shape = (2,))

        self.controlP_distance = None
        self.controlI_distance = None
        self.controlD_distance = None
        if I_DISTANCE:
            self.integral_distance = 0.0
            self.newIntegral_distance = 0.0
        else:
            self.integral_distance = None
            self.newIntegral_distance = None
        if D_DISTANCE:
            self.controlD_distance = 0.0

        self.controlP_angle = None
        self.controlI_angle = None
        self.controlD_angle = None
        if I_ANGLE:
            self.integral_angle = 0.0
            self.newIntegral_angle = 0.0
        else:
            self.integral_angle = None
            self.newIntegral_angle = None
        if D_ANGLE:
            self.controlD_angle = 0.0


        self.start_time = None
        
        self.last = time.time()
        self.fixed_throttle = cfg.USE_FIXED_THROTTLE
        if self.fixed_throttle:
            throttle = cfg.FIXED_THROTTLE
        self.log_file_name = f"logs/steering_controller/{log_file}.log"
        file = open(self.log_file_name, "w")
        file.write("Track: \n")
        file.write(json.dumps(list(self.track.exterior.coords)))
        file.write("\n----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n")
        file.write("Distance Error Parameters: \tP=" + str(P_DISTANCE) + "     kp=" + str(kp_distance) +"     I=" + str(I_DISTANCE) + "     ki=" + str(ki_distance) + "     D=" + str(D_DISTANCE) + "     kd=" + str(kd_distance) + "\n")
        file.write("Angle Error Parameters: \tP=" + str(P_ANGLE) + "     kp=" + str(kp_angle) +"     I=" + str(I_ANGLE) + "     ki=" + str(ki_angle) + "     D=" + str(D_ANGLE) + "     kd=" + str(kd_angle) + "\n")
        file.write("Other Parameters: \t\t\tnext=" + str(NEXT_DISTANCE) + "     correction=" + str(CORRECTION))
        if self.fixed_throttle:
            file.write("     fixed_throttle=" + str(throttle))
        file.write("\n----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n")
        file.write("Distance error\t\t\tAngle error\t\t\tControl\t\t\t\tSteering\t\t\tReceived position\t\t\t\tReceived rotation\t\t\t\t\t\t\tCar direction (angle)\tTrack direction (angle)\tClosest point\t\t\t\t\tNext point\t\t\t\t\t\tDistance P-Control\tDistance NewIntegral\tDistance IntegralTot\tDistance I-Control\tDistance D-Control\tDistance TotControl\tAngle P-Control\tAngle NewIntegral\tAngle IntegralTot\tAngle I-Control\tAngle D-Control\tAngle Control\tTime stamp\tDt_control\n")
        file.close()

        self.on = True
        logger.info("Starting steering_controller (custom)")

        self.graph = show_graph
        if self.graph:
            self.count_messages_send = 0

            # create a UDP socket to send the steering values to the laptop
            self.server_address = ('0.0.0.0', 12345)
            self.udp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_server_socket.bind(self.server_address)

            # Wait for the first message from the receiver
            logger.warning('Waiting to receive the receiver\'s IP address...')
            data, self.receiver_address = self.udp_server_socket.recvfrom(4096)
            receiver_ip = data.decode()
            logger.info(f'Received receiver\'s IP address: {receiver_ip}')
            self.receiver_address = (receiver_ip, 12345)

            # Send the track message
            track_message = json.dumps(list(self.track.exterior.coords)).encode()
            self.udp_server_socket.sendto(track_message, self.receiver_address)
            #print(f'Track sent')


    def send_data(self):

        message_data = {
            "car": {"x": self.position.x, "y": self.position.y},
            "direction": self.car_angle,
            "steer": self.steer,
            "control": self.control,
            "closest-point": {"x": self.closest_point.x, "y": self.closest_point.y},
            "next-closest-point": {"x": self.next_closest_point.x, "y": self.next_closest_point.y},
            "track-tangent": self.track_angle
        }
        self.count_messages_send += 1
        #print(f"{self.count_messages_send} {message_data}")
        message = json.dumps(message_data).encode()
        self.udp_server_socket.sendto(message, self.receiver_address)

    #def update(self):
    #    while self.on:
    #        # Compute the next steering angle given the actual and next position
    #        
    #        #time.sleep(0.052)

    def update_steer(self):
        '''
        Read the new data received (optitrack)
        and calculate the new steering angle to reach the desireed point.
        The message should contain 'position' & 'rotation' keys

        In case of missing values, the steering value remain the same as the last one.
        '''
        try:
            end_time = time.time()

            # Block the sistem values to avoid overwritten while computing
            track = self.track
            position = self.position

            lastError_distance = self.distance_error
            lastError_angle = self.angle_error
            if I_DISTANCE:
                integral_distance = self.integral_distance
            if I_ANGLE:
                integral_angle = self.integral_angle
            if D_DISTANCE:
                controlD_distance = self.controlD_distance
            if D_ANGLE:
                controlD_angle = self.controlD_angle

            # Given the track and our position, we find the nearest next point and the current error from the track
            track_boundary = track.boundary
            closest_point = track_boundary.interpolate(track_boundary.project(position)) 
            next_closest_point = track_boundary.interpolate(track_boundary.project(position) + NEXT_DISTANCE)
            distance_error = position.distance(closest_point)

            # Considering a clockwise movement, if we are inside the track, we need to turn left, if we are outside the track we need to turn right
            if not track.contains(position):
                distance_error *= -1
            
            track_angle = np.degrees(np.arctan2((next_closest_point.y - closest_point.y), (next_closest_point.x - closest_point.x)))


            angle_error = self.angle_difference_degrees(track_angle, self.car_angle)
            #error_distance = distance_error * DISTANCE_COEFF - angle_error * ORIENTATION_COEFF

            # PID on distance error
            if not P_DISTANCE and not I_DISTANCE and not D_DISTANCE:
                control_distance = distance_error
            else:
                control_distance = 0

            if P_DISTANCE:
                controlP_distance = kp_distance * distance_error
                control_distance += controlP_distance

            if I_DISTANCE:
                if self.start_time is None:
                    self.start_time = end_time

                newIntegral_distance = scipy.integrate.trapezoid(
                    [lastError_distance, distance_error], 
                    [self.start_time, end_time]
                )

                integral_distance += newIntegral_distance
                controlI_distance = ki_distance * integral_distance
                control_distance += controlI_distance

            if D_DISTANCE:
                if self.start_time is None:
                    self.start_time = end_time
                
                if self.start_time != end_time:
                    derivate_distance = (distance_error - lastError_distance)/(end_time-self.start_time)
                    controlD_distance = kd_distance * derivate_distance
                    control_distance += controlD_distance

            # PID on angle error
            if not P_ANGLE and not I_ANGLE and not D_ANGLE:
                control_angle = angle_error
            else:
                control_angle = 0

            if P_ANGLE:
                controlP_angle = kp_angle * angle_error
                control_angle += controlP_angle

            if I_ANGLE:
                if self.start_time is None:
                    self.start_time = end_time

                newIntegral_angle = scipy.integrate.trapezoid(
                    [lastError_angle, angle_error], 
                    [self.start_time, end_time]
                )

                #if abs(angle_error) < 2 and abs(distance_error) < 0.15:
                #    integral_angle = 0.0

                integral_angle += newIntegral_angle
                controlI_angle = ki_angle * integral_angle
                control_angle += controlI_angle

            if D_ANGLE:
                if self.start_time is None:
                    self.start_time = end_time
                
                if self.start_time != end_time:
                    derivate_angle = (angle_error - lastError_angle)/(end_time-self.start_time)
                    controlD_angle = kd_angle * derivate_angle
                    control_angle += controlD_angle

            control = control_distance - control_angle
            steer = min(max(control, -1.0), 1.0)
            if CORRECTION_STEER_SIGN:
                if np.sign(steer) == np.sign(angle_error):
                    steer = steer - 0.05 * np.sign(angle_error)

            if CORRECTION and np.sign(steer) == np.sign(angle_error):
                steer = 0.0

            # Save the values back
            self.closest_point = closest_point
            self.next_closest_point = next_closest_point
            self.distance_error = distance_error
            self.track_angle = track_angle
            self.angle_error = angle_error
            self.steer = steer
            self.control = control
            self.control_distance = control_distance
            self.control_angle = control_angle

            if P_DISTANCE:
                self.controlP_distance = controlP_distance
            if I_DISTANCE:
                self.controlI_distance = controlI_distance
                self.integral_distance = integral_distance
                self.newIntegral_distance = newIntegral_distance
            if D_DISTANCE:
                self.controlD_distance = controlD_distance

            if P_ANGLE:
                self.controlP_angle = controlP_angle
            if I_ANGLE:
                self.controlI_angle = controlI_angle
                self.integral_angle = integral_angle
                self.newIntegral_angle = newIntegral_angle
            if D_ANGLE:
                self.controlD_angle = controlD_angle

            self.start_time = time.time()

        except Exception as e:
            print("Error in steering_control:   " + repr(e))

       
    def angle_difference_degrees(self, first_degrees, second_degrees):
        '''
        Calculate the difference in degrees between two angles, accounting for the possibility of a sudden drop from 360 to 0 degrees.
        ''' 

        diff = first_degrees - second_degrees

        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        return diff
    
    '''
    :param position Position of the car
    :param rotation Euler angle of the car 
    :return self.steer Calculated Steering [-1, 1]
    :return self.mode ...
    '''
    def run(self, recv_position: 'np.array', recv_rotation: float, 
    mode=None):
        self.mode = mode if mode is not None else 'user'
        self.position = Point(recv_position[0], recv_position[2])
        self.car_angle = recv_rotation
        
        self.update_steer()

        #if self.debug: print(f"{__name__}: {message}")
        if self.debug: 
            '''
            print(str(self.distance_error) +\
            "  " + str(self.angle_error) + "  " + str(self.steer) +\
            "  " + str(self.position) + "  " +\
            str(self.euler_angles), end='\r')
            '''
            
            print('{:.3f} {:.3f} {:.3f} {:.3f} {:.3f}, {}, {}'.format(
                self.distance_error,
                self.track_angle,
                self.car_angle,
                self.angle_error,
                self.steer,
                self.closest_point,
                self.position
            ), end="\r")
            sys.stdout.flush()
            #print(f"Control: {self.control}\t\tSteering: {self.steer}")
        
        try:
            # Log data on file
            now = time.time()
            with open(self.log_file_name, 'a') as f:
                f.write(str(round(self.distance_error,18)) + " \t" + \
                        str(round(self.angle_error,16)) + " \t" + \
                        str(round(self.control,16)) + " \t" + \
                        str(round(self.steer,16)) + " \t" + \
                        str(self.position) + " \t" + \
                        str(self.car_angle) + " \t\t" + \
                        str(self.track_angle) + " \t\t\t\t\t" + \
                        str(self.closest_point) + "\t" + \
                        str(self.next_closest_point) + " \t" + \
                        str(self.controlP_distance) + " \t" + \
                        str(self.newIntegral_distance) + " \t" + \
                        str(self.integral_distance) + " \t" + \
                        str(self.controlI_distance) + " \t" + \
                        str(self.controlD_distance) + " \t" + \
                        str(self.control_distance) + " \t" + \
                        str(self.controlP_angle) + " \t" + \
                        str(self.newIntegral_angle) + " \t" + \
                        str(self.integral_angle) + " \t" + \
                        str(self.controlI_angle) + " \t" + \
                        str(self.controlD_angle) + " \t" + \
                        str(self.control_angle) + " \t" + \
                        str(now) + "\t" + \
                        str(now - self.last) + "\t"\
                        + "\n")
            self.last = now
        
        except Exception as e:
            if self.debug: 
                print('-- SteeringController --', e)
        
        self.ret_closest_point[0] = self.closest_point.x
        self.ret_closest_point[1] = self.closest_point.y
        self.ret_next_closest_point[0] = self.next_closest_point.x
        self.ret_next_closest_point[1] = self.next_closest_point.y

        return self.steer, self.ret_closest_point, self.track_angle
        
    def shutdown(self):
        self.on = False
        if self.graph:
            self.udp_server_socket.close()
        logger.info(f"Terminating steering_controller (custom)")
        
        time.sleep(.5)
