#!/usr/bin/python3

# Based on example.py from https://github.com/tatobari/hx711py
# and https://github.com/odriverobotics/ODrive/blob/master/tools/odrive_demo.py

from __future__ import print_function

import odrive
from odrive.enums import *
from odrive.utils import *
import sys
import csv
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
ax = my_drive.axis0

# Calibrate motor and wait for it to finish
print("starting calibration...")
ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while ax.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
# ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
# To read a value, simply read the property
print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
val = hx.get_weight(1)
# start_liveplotter(lambda:[ax.encoder.vel_estimate,\
#     ax.controller.vel_setpoint,\
#     ax.motor.current_control.Iq_setpoint,\
#     val/1000.0])
ax.controller.input_vel = 10.0
t0 = time.monotonic()
data = []
data.append(["# Test started at " + time.asctime()])
data.append(["time [s]","velocity setpoint [Hz]",\
    "velocity measured [Hz]","motor current [A]",\
    "load cell weight [g]",\
    "motor torque [Nm]",\
    "brake torque [Nm]"])
kt = ax.motor.config.torque_constant
while True:
    try:        
        weight = hx.get_weight(1)
        # print('weight: {}'.format(weight))
        row = [time.monotonic() - t0,\
            ax.controller.vel_setpoint,\
            ax.encoder.vel_estimate,\
            ax.motor.current_control.Iq_measured,\
            -weight,\
            ax.motor.current_control.Iq_measured*kt,\
            weight*-0.001*9.81*5*2.54/100]
        data.append(row)

        # time.sleep(0.01)

    except (KeyboardInterrupt, SystemExit):
        print("Cleaning...")
        GPIO.cleanup()
        ax.requested_state = AXIS_STATE_IDLE
        with open(time.asctime() + ".csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            csvWriter.writerows(data)
        print("Bye!")
        sys.exit()
    
    except :
        ax.requested_state = AXIS_STATE_IDLE


