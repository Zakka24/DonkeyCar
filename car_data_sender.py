#!python3

import sys
import socket
import json

### MAIN CLASS ###

# Computes Speed from Optitrack data
'''
Send Car Data to Broadcast
'''
class CarDataSender:
    def __init__(self, donkey_id: int, v2x_port: int, 
    debug:bool = False) -> None:
        
        self.donkey_id = donkey_id
        self.debug = debug

        # Init socket
        self.s = socket.socket(socket.AF_INET, 
        socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.s_port = v2x_port

    def run_threaded(self, receiver_position: list, current_speed:float, 
    receiver_rotation: float, sc_steering: float, sc_closest_point: list, 
    sc_track_tangent: float):
        self.run(receiver_position, current_speed, receiver_rotation, 
        sc_steering, sc_closest_point, sc_track_tangent)

    def run(self, receiver_position: list, current_speed: float, 
    receiver_rotation: float, sc_steering: float, sc_closest_point: list, 
    sc_track_tangent: float):
        msg = { 
            'id': self.donkey_id,
            'car': {
                'x': receiver_position[0], 
                'y': receiver_position[2]
            },
            'speed': current_speed,
            'direction': receiver_rotation,
            'steer': sc_steering,
            'closest-point': {
                'x': sc_closest_point[0], 
                'y': sc_closest_point[1]
            },
            'track-tangent': sc_track_tangent
        }
        
        b_msg = json.dumps(msg).encode(encoding='utf-8')

        self.s.sendto(b_msg, ('<broadcast>', self.s_port))
    
    def shutdown(self) -> None:
        self.s.close()

    def update(self) -> None:
        pass
