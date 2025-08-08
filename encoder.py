"""
ArduinoEncoder() is working, coded by Amigoni

The other classes deprecated in favor on donkeycar.parts.Tachometer and donkeycar.parts.Odometer.

Encoders and odometry
"""
'''
from datetime import datetime
from donkeycar.utilities.deprecated import deprecated
import re
import time
from sys import stdout

import logging
logger = logging.getLogger(__name__)

# The Arduino class is for a quadrature wheel or motor encoder that is being read by an offboard microcontroller
# such as an Arduino or Teensy that is feeding serial data to the RaspberryPy or Nano via USB serial. 
# The microcontroller should be flashed with this sketch (use the Arduino IDE to do that): https://github.com/zlite/donkeycar/tree/master/donkeycar/parts/encoder/encoder
# Make sure you check the sketch using the "test_encoder.py code in the Donkeycar tests folder to make sure you've got your 
# encoder plugged into the right pins, or edit it to reflect the pins you are using.

# You will need to calibrate the mm_per_tick line below to reflect your own car. Just measure out a meter and roll your car
# along it. Change the number below until it the distance reads out almost exactly 1.0 

# This samples the odometer at 10HZ and does a moving average over the past ten readings to derive a velocity

class ArduinoEncoder():  # use an Arduino or Teensy to read the encoder, sending a ticks counter via USB serial
    def __init__(self,cfg,debug=False,baud=9600,log_file = "encoder"):
        """
        Rotary Encoder
        """

        import serial
        import serial.tools.list_ports
        logger.info("Initializing Encoder")
        if debug:
            for item in serial.tools.list_ports.comports():
                print('Serial port: ' + str(item))
        #self.ser = serial.Serial('/dev/cu.usbmodem14401', baud, 8, 'N', 1, timeout=1)
        self.ser = serial.Serial('/dev/ttyACM0', baud, 8, 'N', 1, timeout=1)
        # initialize the odometer values
        self.m_per_tick = cfg.MM_PER_TICK / 1000.0
        self.poll_delay = cfg.POLL_DELAY
        self.meters = 0
        self.last_time = time.time()
        self.meters_per_second = 0
        self.counter = 0
        self.on = True
        self.debug = debug
        self.top_speed = 0
        self.prev_dist = 0.
        self.ser.write(str.encode('r\r'))

        self.log_file_name = f"logs/encoder/{log_file}.log"
        self.file_speed = open(self.log_file_name, "w")

    def update(self):

        # keep looping infinitely until the thread is stopped
        while self.on:

            input = ''
            while self.ser.in_waiting > 0:   # read the serial port and see if there's any data there
                buffer = self.ser.readline()
                input = buffer.decode('ascii')
            self.ser.write(str.encode('p\r'))  # queue up another reading by sending the "p" character to the Arduino
            input = self.ser.readline()
            input = input.decode('ascii')
            if input != '':
                temp = input.strip()  # remove any whitespace
                try:
                    ticks = int(temp)
                except Exception as e:
                    ticks = self.counter
            else:
                ticks = self.counter
            #save the ticks and reset the counter
            # try:
            #     self.ser.write(str.encode('p'))
            #     d = self.ser.readline()
            #     print(d)
            #     ticks = int(d.decode())
            # except:
            #     ticks = 0

            # save off the last time interval and reset the timer
            start_time = self.last_time
            end_time = time.time()
            self.last_time = end_time

            # calculate elapsed time and distance traveled
            seconds = end_time - start_time
            distance = (ticks - self.counter) * self.m_per_tick
            velocity = distance / seconds

            # update the odometer values
            self.meters += distance
            if(self.meters_per_second != velocity):
                self.meters_per_second = velocity
                #stdout.write("\rvelocity (m/s): {:.3f}".format(self.meters_per_second))
                stdout.flush()
            if self.meters_per_second > self.top_speed:
                self.top_speed = self.meters_per_second
            
            #write velocity on log file
            self.file_speed.write(datetime.now().strftime('%Y-%m-%d %H:%M:%S - '))
            self.file_speed.write("Velocity: {:.3f} m/s\n".format(self.meters_per_second))

            # console output for debugging
            if self.debug:
                stdout.write("\rvelocity (m/s): {:.3f}, distance (m): {:.3f}".format(self.meters_per_second, self.meters))
                stdout.flush()
                print('seconds:', seconds)
                print('distance:', distance)
                print('velocity:', velocity)

                print('distance (m):', round(self.meters, 4))
                print('velocity (m/s):', self.meters_per_second)
            self.counter = ticks
            
            time.sleep(self.poll_delay)

    def run_threaded(self):
        delta = self.meters - self.prev_dist
        self.prev_dist = self.meters
        return self.meters, float(self.meters_per_second), delta

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        logger.info(f'Stopping Rotary Encoder\n**************************************\n  Distance Travelled: {round(self.meters, 4)} meters \n  Top Speed: {round(self.top_speed, 4)} meters/second\n**************************************')
        time.sleep(.5)
        self.file_speed.close()
        time.sleep(.5)


#ser = serial.Serial('/dev/cu.usbmodem14401', 9600, 8, 'N', 1, timeout=0)
#while True:
#    ser.write(str.encode("p\r"))
#    d = ser.readline()
#    print(d)
#    input("press key")


#enc = ArduinoEncoder(mm_per_tick=1, poll_delay=0.01, debug=True)
#enc.update()


@deprecated("Deprecated as unused")
class AStarSpeed:
    def __init__(self):
        from donkeycar.parts.teensy import TeensyRCin
        self.speed = 0
        self.linaccel = None
        self.sensor = TeensyRCin(0)
        self.on = True

    def update(self):
        encoder_pattern = re.compile('^E ([-0-9]+)( ([-0-9]+))?( ([-0-9]+))?$')
        linaccel_pattern = re.compile('^L ([-.0-9]+) ([-.0-9]+) ([-.0-9]+) ([-0-9]+)$')

        while self.on:
            start = datetime.now()

            l = self.sensor.astar_readline()
            while l:
                m = encoder_pattern.match(l.decode('utf-8'))

                if m:
                    value = int(m.group(1))
                    # rospy.loginfo("%s: Receiver E got %d" % (self.node_name, value))
                    # Speed
                    # 40 ticks/wheel rotation,
                    # circumfence 0.377m
                    # every 0.1 seconds
                    if len(m.group(3)) > 0:
                        period = 0.001 * int(m.group(3))
                    else:
                        period = 0.1

                    self.speed = 0.377 * (float(value) / 40) / period   # now in m/s
                else:
                    m = linaccel_pattern.match(l.decode('utf-8'))

                    if m:
                        la = { 'x': float(m.group(1)), 'y': float(m.group(2)), 'z': float(m.group(3)) }

                        self.linaccel = la
                        print("mw linaccel= " + str(self.linaccel))

                l = self.sensor.astar_readline()

            stop = datetime.now()
            s = 0.1 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

    def run_threaded(self):
        return self.speed # , self.linaccel

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping AStarSpeed')
        time.sleep(.5)


@deprecated("Deprecated in favor of donkeycar.parts.tachometer.Tachometer(GpioEncoder)")
class RotaryEncoder():
    def __init__(self, mm_per_tick=0.306096, pin=13, poll_delay=0.0166, debug=False):
        import pigpio
        self.pi = pigpio.pi()
        self.pin = pin
        self.pi.set_mode(self.pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin, pigpio.PUD_DOWN)
        self.cb = self.pi.callback(self.pin, pigpio.FALLING_EDGE, self._cb)


        # initialize the odometer values
        self.m_per_tick = mm_per_tick / 1000.0
        self.poll_delay = poll_delay
        self.meters = 0
        self.last_time = time.time()
        self.meters_per_second = 0
        self.counter = 0
        self.on = True
        self.debug = debug
        self.top_speed = 0
        self.prev_dist = 0.

    def _cb(self, pin, level, tick):
        self.counter += 1

    def update(self):
        # keep looping infinitely until the thread is stopped
        while(self.on):
                
            #save the ticks and reset the counter
            ticks = self.counter
            self.counter = 0
            
            #save off the last time interval and reset the timer
            start_time = self.last_time
            end_time = time.time()
            self.last_time = end_time
            
            #calculate elapsed time and distance traveled
            seconds = end_time - start_time
            distance = ticks * self.m_per_tick
            velocity = distance / seconds
            
            #update the odometer values
            self.meters += distance
            self.meters_per_second = velocity
            if(self.meters_per_second > self.top_speed):
                self.top_speed = self.meters_per_second

            #console output for debugging
            if(self.debug):
                print('seconds:', seconds)
                print('distance:', distance)
                print('velocity:', velocity)

                print('distance (m):', round(self.meters, 4))
                print('velocity (m/s):', self.meters_per_second)

            time.sleep(self.poll_delay)

    def run_threaded(self, throttle):
        self.prev_dist = self.meters
        return self.meters_per_second

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('Stopping Rotary Encoder')
        print("\tDistance Travelled: {} meters".format(round(self.meters, 4)))
        print("\tTop Speed: {} meters/second".format(round(self.top_speed, 4)))
        if self.cb != None:
            self.cb.cancel()
            self.cb = None
        self.pi.stop()
'''
"""
Rotary Encoder
"""
import os
from datetime import datetime
import re
import time
from sys import stdout

import serial

import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class ArduinoEncoder():
    def __init__(self, debug=False, baud=9600):
        """
        Rotary Encoder
        """

        import serial
        import serial.tools.list_ports
        logger.info("Initializing Encoder")
        if debug:
            for item in serial.tools.list_ports.comports():
                print('Serial port: ' + str(item))
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baud, timeout=1)
        except serial.SerialException as e:
            logger.error(f"Error opening serial port: {e}")
            self.ser = None
            self.on = False
            return

        self.m_per_tick = 2 * 38.0 * 3.14 / 400 / 1000.0
        self.poll_delay = 0.01
        self.meters = 0
        self.last_time = time.time()
        self.speed = 0
        self.old_speed = 0.0
        self.acceleration = 0.0
        self.counter = 0
        self.on = True
        self.debug = debug
        self.top_speed = 0
        self.prev_dist = 0.0
        self.initial_reading = False

        log_dir = 'logs/encoder'
        os.makedirs(log_dir, exist_ok=True)
        self.file_speed = open(datetime.now().strftime(f'{log_dir}/%Y%m%d_%H%M%S.log'), "w")

    def update(self):
        if not hasattr(self, 'current_ticks') or not hasattr(self, 'current_arduino_time_ms') or not self.initial_reading:
            return  # Wait for the first reading in run_threaded

        try:
            ticks = self.current_ticks
            arduino_time_ms = self.current_arduino_time_ms

            delta_ticks = ticks - self.counter
            self.counter = ticks

            delta_time_s = (arduino_time_ms - self.last_arduino_time_ms) / 1000.0
            if delta_time_s > 0:
                distance = delta_ticks * self.m_per_tick
                velocity = distance / delta_time_s
                self.meters += distance
                self.old_speed = self.speed
                self.speed = velocity
                self.acceleration = (self.old_speed - self.speed) / delta_time_s
                if self.speed > self.top_speed:
                    self.top_speed = self.speed
                self.file_speed.write(datetime.now().strftime('%Y-%m-%d %H:%M:%S - '))
                self.file_speed.write(
                    f"Acceleration: {self.acceleration:.3f} m/s^2, Velocity: {self.speed:.3f} m/s, Distance: {self.meters:.3f} m\n")
                if self.debug:
                    stdout.write(
                        f"\racceleration (m/s^2): {self.acceleration:.3f}, velocity (m/s): {self.speed:.3f}, distance (m): {self.meters:.3f}")
                    stdout.flush()
            self.last_arduino_time_ms = arduino_time_ms
            self.last_time = time.time()

        except Exception as e:
            logger.error(f"Error in update: {e}")
            self.on = False

    def run_threaded(self):
        if not self.on or not self.ser:
            return self.meters, float(self.speed), (self.meters - self.prev_dist), float(self.acceleration)

        try:
            self.ser.write(b'p\r')  # Request data from Arduino
            time.sleep(0.1)  # Small delay to allow Arduino to respond
            input_bytes = self.ser.readline()
            input_str = input_bytes.decode('ascii').strip()
            if input_str:
                if self.debug:
                    print(f"Received from Arduino: '{input_str}'")
                try:
                    parts = input_str.split(',')
                    self.current_ticks = int(parts[0])
                    self.current_arduino_time_ms = int(parts[1])
                    if not self.initial_reading:
                        self.counter = self.current_ticks  # Initialize counter on first read
                        self.last_arduino_time_ms = self.current_arduino_time_ms
                        self.initial_reading = True
                   
                    self.update()
                except ValueError:
                    logger.warning(f"Invalid data format from Arduino: '{input_str}'")
            else:
                if self.debug:
                    print("No data received from Arduino")
        except serial.SerialException as e:
            logger.error(f"Serial communication error: {e}")
            self.on = False
        except Exception as e:
            logger.error(f"Error in run_threaded: {e}")
            self.on = False

        delta = self.meters - self.prev_dist
        self.prev_dist = self.meters
        return self.meters, float(self.speed), delta, float(self.acceleration)

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        logger.info(f'Stopping Rotary Encoder\n**************************************\n  Distance Travelled: {round(self.meters, 4)} meters \n  Top Speed: {round(self.top_speed, 4)} meters/second\n**************************************')
        time.sleep(.5)
        self.file_speed.close()
        time.sleep(.5)


#ser = serial.Serial('/dev/cu.usbmodem14401', 9600, 8, 'N', 1, timeout=0)
#while True:
#    ser.write(str.encode("p\r"))
#    d = ser.readline()
#    print(d)
#    input("press key")


#ser = serial.Serial('/dev/cu.usbmodem14401', 9600, 8, 'N', 1, timeout=0)
#while True:
#    ser.write(str.encode("p\r"))
#    d = ser.readline()
#    print(d)
#    input("press key")
enc = ArduinoEncoder(debug=True)
enc.update()
