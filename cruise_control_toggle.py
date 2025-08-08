#!python3

### MAIN CLASS ###

# Computes Speed from Optitrack data
class CruiseControlToggle:
    '''
    Toggle between Manual Throttle and Cruise Control Throttle
    '''
    def __init__(self, debug = False):
        self.debug = debug

    def run_threaded(self, manual_throttle: float, cc_throttle: float,
    manual_cc_toggle: bool = False) -> float:
        return self.run(manual_throttle, cc_throttle, manual_cc_toggle)

    def run(self, manual_throttle: float, cc_throttle: float,
    manual_cc_toggle: bool = False) -> float:
        if not manual_cc_toggle:
            if self.debug:
                print('-- CC Toggle -- Manual Throttle:', manual_throttle)
            return manual_throttle
        else:
            if self.debug:
                print('-- CC Toggle -- CC Throttle:', cc_throttle)
            return cc_throttle
