#!python

import socket
import time
import json

### CONSTANTS ###
LISTEN_PERIOD: float = 0.01
MSG_SIZE: int = 1024
#################

class V2VListener:
    def __init__(self, donkey_id: int, v2v_listen_port: int, debug: bool):
        self.debug = debug
        self.donkey_id = donkey_id
        self.v2v_listen_port = v2v_listen_port

        self.stop = False
        self.give_prec = []
        self.take_prec = []

        self.run = True

    def update(self):
        listener_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener_s.bind(('', self.v2v_listen_port))
        
        while self.run:
            try:
                msg_b = listener_s.recv(MSG_SIZE)
                msg = json.loads(msg_b.decode('utf-8'))
                
                if self.debug:
                    print('-- V2V Listener --', msg)

                if ('id' in msg and int(msg['id']) > 0 and
                    msg['id'] != self.donkey_id and
                    'msg' in msg):
                    
                    if self.debug:
                        print('-- V2V Listener -- We are in!')
                    
            
                    if msg['msg'] == 'STOP':
                        self.stop = True
                    elif msg['msg'] == 'GIVE PREC': 
                        self.give_prec += [str(int(msg['id']))]
                    elif msg['msg'] == 'TAKE PREC':
                        self.take_prec += [str(int(msg['id']))]

            except Exception as e:
                if self.debug:
                    print('-- V2V Listener --', e)

            time.sleep(LISTEN_PERIOD)
        
        listener_s.close()

    def run_threaded(self):
        stop_res = self.stop
        give_prec_res = self.give_prec
        take_prec_res = self.take_prec

        # Init for next call
        self.stop = False
        self.give_prec = []
        self.take_prec = []
        
        if self.debug:
            print('-- V2V Listener --\tGive Prec:', give_prec_res,
                '\tTake Prec:', take_prec_res)

        return stop_res, str(give_prec_res), str(take_prec_res)


    def shutdown(self):
        time.sleep(.5)
        self.run = False
        time.sleep(.5)
