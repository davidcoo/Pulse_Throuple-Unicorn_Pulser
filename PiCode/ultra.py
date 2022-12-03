import RPi.GPIO as GPIO                                                                                                         
import sys
import time                                                             
import signal
import random
import sysv_ipc
GPIO.setmode (GPIO.BCM)                                                 

TRIG_FRONT = 17
ECHO_FRONT = 27
TRIG_LEFT = 5
ECHO_LEFT = 6
TRIG_RIGHT = 20                                                               # Trigger output for the ultrasonic sensor
ECHO_RIGHT = 21                                                               # Echo return from the ultrasonic sensor


SHM_KEY_RIGHT = 0x1234
SHM_KEY_LEFT = 0x5678
SHM_KEY_FRONT  = 0x1357

# can use (17, 27), (5, 6), (20, 21)

# source code: https://forums.raspberrypi.com/viewtopic.php?t=77534 #godbless
def get_distance (trig_t, echo_t):
    trig = trig_t
    echo = echo_t
    if GPIO.input (echo):                                               # If the 'Echo' pin is already high
        return (100)                                                    # then exit with 100 (sensor fault)

    distance = 0                                                        # Set initial distance to zero

    GPIO.output (trig,False)                                            # Ensure the 'Trig' pin is low for at
    time.sleep (0.05)                                                   # least 50mS (recommended re-sample time)

    GPIO.output (trig,True)                                             # Turn on the 'Trig' pin for 10uS (ish!)
    dummy_variable = 0                                                  # No need to use the 'time' module here,
    dummy_variable = 0                                                  # a couple of 'dummy' statements will do fine
    
    GPIO.output (trig,False)                                            # Turn off the 'Trig' pin
    time1, time2 = time.time(), time.time()                             # Set inital time values to current time
    
    while not GPIO.input (echo):                                        # Wait for the start of the 'Echo' pulse
        time1 = time.time()                                             # Get the time the 'Echo' pin goes high
        if time1 - time2 > 0.02:                                        # If the 'Echo' pin doesn't go high after 20mS
            distance = 100                                              # then set 'distance' to 100
            break                                                       # and break out of the loop
        
    if distance == 100:                                                 # If a sensor error has occurred
        return (distance)                                               # then exit with 100 (sensor fault)
    
    while GPIO.input (echo):                                            # Otherwise, wait for the 'Echo' pin to go low
        time2 = time.time()                                             # Get the time the 'Echo' pin goes low
        if time2 - time1 > 0.02:                                        # If the 'Echo' pin doesn't go low after 20mS
            distance = 100                                              # then ignore it and set 'distance' to 100
            break                                                       # and break out of the loop
        
    if distance == 100:                                                 # If a sensor error has occurred
        return (distance)                                               # then exit with 100 (sensor fault)
        
                                                                        # Sound travels at approximately 2.95uS per mm
                                                                        # and the reflected sound has travelled twice
                                                                        # the distance we need to measure (sound out,
                                                                        # bounced off object, sound returned)
                                                                        
    distance = (time2 - time1) / 0.00000295 / 2 / 10                    # Convert the timer values into centimetres
    return (distance)                                                   # Exit with the distance in centimetres

def signal_handler(sig, frame):
    # set the write->complete to 1? 
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

from ctypes import *
import os

so_file = os.path.join(os.path.dirname(__file__), "linkc.so")
linkc = CDLL(so_file)

GPIO.setwarnings(False)
sides = sys.argv[1: ]

active_sensors = []
if "ALL" not in sides and "LEFT" not in sides and "RIGHT" not in sides and "FRONT" not in sides:
    print("Please enter a valid sensor direction.")
    sys.exit()
if "ALL" in sides: 
    sides = ["LEFT", "RIGHT", "FRONT"]
if "LEFT" in sides: 
    GPIO.setup(TRIG_LEFT, GPIO.OUT)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
if "RIGHT" in sides:
    linkc.set_up(SHM_KEY_RIGHT)
    GPIO.setup(TRIG_RIGHT, GPIO.OUT)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
if "FRONT" in sides:
    GPIO.setup(TRIG_FRONT, GPIO.OUT)
    GPIO.setup(ECHO_FRONT, GPIO.IN)

# can edit this if you're running this three seperate times

while(1):
    if "LEFT" in sides:
        linkc.set_up(SHM_KEY_LEFT)
        l = get_distance(TRIG_LEFT, ECHO_LEFT)
        l = int(l)
        linkc.write_vals(bytes(str(l).encode("ascii")))
    if "RIGHT" in sides:
        linkc.set_up(SHM_KEY_RIGHT)
        r = get_distance(TRIG_RIGHT, ECHO_RIGHT)
        r = int(r)
        linkc.write_vals(bytes(str(r).encode("ascii")))
    if "FRONT" in sides: 
        linkc.set_up(SHM_KEY_FRONT)
        f = get_distance(TRIG_FRONT, ECHO_FRONT)
        f = int(f)
        linkc.write_vals(bytes(str(f).encode("ascii")))
    time.sleep(0.5)
    