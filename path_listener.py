#!python

import socket
import json
import base64
import pickle

### CONSTANTS ###
MSG_MAX_LEN: int = 65536
#################

class PathListener:
    def __init__(self, path_sharing_port: int, debug: bool = False):
        self.debug = debug
        
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind(('', path_sharing_port))
        
    def listen(self):
        car_id = -1
        car_path = None
        
        # Listen
        try:
            msg = self.server.recv(MSG_MAX_LEN).decode('ascii')
            
            #if self.debug:
            #    print(msg)

            msg_dict = json.loads(msg)
            car_id = msg_dict['id']
            assert type(car_id) == int
            
            base64_bin = msg_dict['data'].encode('ascii')
            car_path_bin = base64.b64decode(base64_bin)

            car_path = pickle.loads(car_path_bin)

        except Exception as e:
            if self.debug:
                print('-- Path Listener --', e)
        
        return str(car_id), car_path

    def shutdown(self):
        self.server.close()
