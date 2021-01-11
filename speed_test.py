#!/usr/bin/python3

# Based on example.py from https://github.com/tatobari/hx711py
# and https://github.com/odriverobotics/ODrive/blob/master/tools/odrive_demo.py

from __future__ import print_function

import odrive
from odrive.enums import *
import sys
import time
import math
import RPi.GPIO as GPIO
from hx711 import HX711

referenceUnit = 22*876/881

def cleanAndExit():
    print("Cleaning...")
    GPIO.cleanup()
    print("Bye!")
    sys.exit()

hx = HX711(5, 6)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(referenceUnit)
hx.reset()

hx.tare()

print("Tare done! Add weight now...")

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any()

# Calibrate motor and wait for it to finish
print("starting calibration...")
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
# To read a value, simply read the property
print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

t0 = time.monotonic()
while True:
    try:        
        val = hx.get_weight(5)
        print('val: {}'.format(val))

        # hx.power_down()
        # hx.power_up()

        setpoint = math.sin((time.monotonic() - t0)/2)
        print("goto " + str(setpoint))
        my_drive.axis0.controller.input_pos = setpoint
        time.sleep(0.01)

    except (KeyboardInterrupt, SystemExit):
        print("Cleaning...")
        GPIO.cleanup()
        my_drive.axis0.requested_state = AXIS_STATE_IDLE
        print("Bye!")
        sys.exit()
    
    # except :
    #     my_drive.axis0.requested_state = AXIS_STATE_IDLE


