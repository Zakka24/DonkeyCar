#!python3

### MAIN CLASS ###

class SteeringControlToggle:
    '''
    Toggle between Manual Steering and Cruise Control Steering
    '''
    
    def __init__(self, debug = False):
        self.debug = debug

    def run_threaded(self, manual_steering: float, sc_steering: float,
    manual_sc_toggle: bool = False) -> float:
        return self.run(manual_steering, sc_steering, manual_sc_toggle)

    def run(self, manual_steering: float, sc_steering: float,
    manual_sc_toggle: bool = False) -> float:
        if not manual_sc_toggle:
            if self.debug:
                print('-- SC Toggle -- Manual Steering:', manual_steering)
            return manual_steering
        else:
            if self.debug:
                print('-- SC Toggle -- SC Steering:', sc_steering)
            return sc_steering
