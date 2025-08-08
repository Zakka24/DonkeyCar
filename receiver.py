"""
Server to receive commands from the laptop
"""

import numpy as np
import socket
import time
import logging
import json

logger = logging.getLogger(__name__)

# KEYS
POS_KEY: str = 'position'
ROT_KEY: str = 'rotation'


class Receiver():

    def __init__(self, cfg, debug=False):

        self.port = cfg.OPTITRACK_RECV_PORT
        self.debug = debug
        self.on = True
        self.poll_delay = cfg.POLL_DELAY
        self.rcvdData = ""
        self.recv_pos = np.ndarray(shape = (3,))
        self.recv_rot = np.ndarray(shape = (4,))

        #self.connect_socket()
        if self.debug:
            print(f'Listening on port {self.port}')
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('0.0.0.0', self.port))

    def connect_socket(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        logger.warning("Waiting for a socket connection...")

        self.s.bind(('0.0.0.0', self.port))
        self.s.listen(5)

        try: 
            self.c, self.addr = self.s.accept()
        except Exception as e:
            logger.error(f"Error in {__name__}:   " + repr(e))


        logger.info(f"Socket Up and running with a connection from {self.addr}")

    def update(self):

        while self.on:
            #self.rcvdData = self.c.recv(1024).decode("utf-8")
            self.rcvdData = self.s.recvfrom(1024)[0].decode('utf-8')
            #print(f"type selfRecvData: {type(self.recvData)}")
            #print(f"self recvData: {self.recvData}")
            #print(f"port: {self.port}")
            #if self.debug: print(type(self.recvData))

            if self.rcvdData:
                if self.debug:
                    print(f"{__name__}: {self.rcvdData}")
                pass
                #time.sleep(0.0001)  #this is the interval of how often the optitrack sends a message

            else:   #client has closed the connection
                #self.connect_socket()
                
                # For UDP
                if self.debug:
                   print(f'Listening on port {self.port}')
                self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.s.bind(('0.0.0.0', self.port))


    def run_threaded(self):
        try:
            assert type(self.rcvdData) is str
            recv_msg_dict = json.loads(self.rcvdData)
            assert type(recv_msg_dict) is dict
            assert POS_KEY in recv_msg_dict.keys()
            assert ROT_KEY in recv_msg_dict.keys()
            

            recv_pos = recv_msg_dict[POS_KEY]

            assert type(recv_pos) is list
            assert len(recv_pos) == 3
            
            
            for i in range(len(recv_pos)):
                assert type(recv_pos[i]) is float
                self.recv_pos[i] = recv_pos[i]
            
            self.recv_pos[2] *= -1

            recv_rot = recv_msg_dict[ROT_KEY]

            assert type(recv_rot) is list
            assert len(recv_rot) == 4
            

            for i in range(len(recv_rot)):
                assert type(recv_rot[i]) is float
                self.recv_rot[i] = recv_rot[i]
            #natnet sends quaternion in the format: vector, scalar (opposite than optitrack)
        except Exception as e:
            
            if self.debug: 
                print('Receiver --- No message or something went wrong.\n', e)

        return self.recv_pos, self.recv_rot

    def shutdown(self):
        logger.info('Stopping Server (Receiver)')
        
        #close the server-socket
        self.c.shutdown(socket.SHUT_RDWR)
        self.c.close()
        self.on = False

        logger.info('Server (Receiver) down')
        time.sleep(.5)

