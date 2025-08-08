#!python

import socket
import struct
import time
import json
import pickle
import base64
import threading

from donkeycar.parts.path_to_be_followed import PathToBeFollowed

### CONSTANTS ###
RESEND_PERIOD: float = 10.0  # Seconds
LISTEN_PERIOD: float = 1.0 # seconds
MSG_LEN: int = 64
#################

class PathSender:
    def __init__(self, cfg, debug: bool = False):
        
        self.cfg = cfg
        self.debug = debug
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip_addrs = {} # Save in string format
        
        # Dump Track as binary file
        track = PathToBeFollowed().get_track()
        track_bin = pickle.dumps(track)
        
        # Encode into bytes base64 format (string)
        track_base64 = base64.b64encode(track_bin).decode('ascii')
        
        msg_str = json.dumps({
            'id': self.cfg.DONKEY_ID,
            'data': track_base64
        })
        
        # Encode into bytes
        self.msg = msg_str.encode('ascii')
        
        # Start thread for Path Request Listening
        self.stop = threading.Event()
        self.listen_t = threading.Thread(target=self.listen)
        self.listen_t.start()
    
    '''
    Listen for PATH REQUESTs and send back the path via UDP
    '''
    def listen(self):
        listen_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listen_s.bind(('', self.cfg.PATH_REQUEST_PORT))
        listen_s.setblocking(False)
        #listen_s.settimeout(1.0)

        while not self.stop.is_set():
            try:    
                msg_b, addr = listen_s.recvfrom(MSG_LEN)
                msg = msg_b.decode('utf-8')

                if msg == 'PATH REQUEST':
                    self.socket.sendto(
                        self.msg,
                        (addr[0], self.cfg.PATH_UNI_SEND_PORT)
                    )

            except BlockingIOError:
                pass

            except Exception as e:
                if self.debug:
                    print('-- Path Sender (Listen) --', e)
            
            self.stop.wait(timeout=LISTEN_PERIOD)

        listen_s.close()

    def update(self):
        while not self.stop.is_set():
            # Re-send to everyone
            for addr in self.ip_addrs.values():
                self.socket.sendto(
                    self.msg, 
                    (addr, self.cfg.PATH_SHARING_PORT)
                )
                
            self.stop.wait(timeout=RESEND_PERIOD)

    '''
    This components stores all the ip addresses that it sent its
    path to. 
    If the Garbage Collector of the component
    V2XListener sends an address to remove, that it is also removed 
    from self.ip_addrs.
    The path is sent every RESEND_PERIOD seconds to everyone in 
    self.ip_addrs
    '''
    def run_threaded(self, close_addrs_str: str, to_remove: 'np.array'):
        try:
            if self.debug:
                print(close_addrs_str)

            assert type(close_addrs_str) == str
            close_addrs = json.loads(close_addrs_str)
            assert type(close_addrs) == dict
            
            # TODO: Other controls over the input

            for k in close_addrs.keys():
                if not k in self.ip_addrs.keys() or\
                    close_addrs[k] != self.ip_addrs[k]:
                    
                    # Add to dictionary
                    self.ip_addrs[k] = close_addrs[k]
                    
                    # Send
                    self.socket.sendto(
                        self.msg, 
                        (self.ip_addrs[k], self.cfg.PATH_SHARING_PORT)
                    )
            # Remove addresses not used anymore
            for k in to_remove: # It's an array
                self.ip_addrs.pop(k, None)  # Remove if in dictinary

        except Exception as e:
            if self.debug:
                print('-- Path Sender --', e)


    def shutdown(self):
        time.sleep(.5)
        self.socket.close()
        self.stop.set()
        self.listen_t.join() # Wait for listening thread
        time.sleep(.5)
