#!python

import socket
import time
import json
import numpy as np

import logging

logger = logging.getLogger(__name__)

### CONSTANTS ###
UPDATE_PERIOD: float = 0.005
MAXIMUM_DISTANCE: float = 4.0
CACHE_DIST_LIMIT: float = 6.0
PACKET_SIZE: float = 1024
GARBAGE_COLLECTOR_PERIOD: int = 500 # Cycles
CACHE_PERMANENCE: float = 10        # seconds
#################

### MAIN CLASS ###
'''
Listen to other car's messages on the network
'''
class V2XListener:
    def __init__(self, donkey_id: int, v2x_port: int, debug: bool = False):
        self.debug = debug
        
        self.donkey_id = donkey_id

        # Open listening socket on broadcast
        self.s = socket.socket(socket.AF_INET, 
            socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s.bind(('', v2x_port))
        self.s.setblocking(False)

        # Dictoinary of other cars
        self.timestamps = {}
        self.close_cars = {}
        self.close_addrs = {}
        self.to_remove = []
        self.position = np.array((0.0, 0.0)) # [x, y]
        self.run = True
        self.n_cycle = 0
    
    # Can be useful when cars leave the area without telling you
    # or when they are turned off within the area
    def garbage_collector(self):
        if self.debug:
            print('-- V2XListener -- Garbage collector')
        self.to_remove = []
        time_limit = CACHE_PERMANENCE
        now = time.time()
        for k in list(self.close_cars.keys()):
            if now - self.timestamps[k] > time_limit:
                self.timestamps.pop(k, None)
                self.close_cars.pop(k, None)
                self.close_addrs.pop(k, None)
                self.to_remove.append(k)

    def update(self):
        car_pos = np.array((0.0, 0.0))
        while self.run:
            try:
                msg_bytes, car_addr = self.s.recvfrom(PACKET_SIZE)
                msg = json.loads(msg_bytes.decode('ascii'))
                
                #if self.debug:
                #    print('-- V2X Listener --', msg)

                car_id = int(msg['id'])

                if car_id != self.donkey_id:
                    car_pos[0] = msg['car']['x']
                    car_pos[1] = msg['car']['y']
                    
                    # If the distance between this car and the one it
                    # received the message from is less that 
                    # MAXIMUM_DISTANCE then proceed
                    distance = np.linalg.norm(self.position - car_pos)
                    if  distance <= MAXIMUM_DISTANCE:
                        body = [list(car_pos), msg['speed'], 
                            msg['direction'], msg['steer']]
                        
                        self.timestamps[car_id] = time.time()
                        self.close_cars[car_id] = body
                        self.close_addrs[car_id] = car_addr[0]

                    elif distance > CACHE_DIST_LIMIT:
                        # Remove from cache
                        self.close_cars.pop(car_id, None) 
            except BlockingIOError:
                pass
            except Exception as e:
                if self.debug:
                    print('-- V2X Listener --', e)
            
            # Call Garbage Collector
            if self.n_cycle == GARBAGE_COLLECTOR_PERIOD - 1:
                self.garbage_collector()

            self.n_cycle = (self.n_cycle + 1) % GARBAGE_COLLECTOR_PERIOD

            time.sleep(UPDATE_PERIOD)

    def run_threaded(self, position: list):
        self.position[0] = position[0]
        self.position[1] = position[1]
        

        close_cars_ret = json.dumps(self.close_cars)
        close_addrs_ret = json.dumps(self.close_addrs)
        to_remove = np.array(self.to_remove)

        # Set speed of every close car to 0.0 in the case there
        # are bugs
        for car_id in self.close_cars.keys():
            self.close_cars[car_id][1] = 0.0
        
        return close_cars_ret, close_addrs_ret, to_remove

    def shutdown(self):
        time.sleep(.5)
        self.run = False
        self.s.close()
        time.sleep(.5)
