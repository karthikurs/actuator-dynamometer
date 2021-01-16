#!/usr/bin/env python3

# Based on example.py from https://github.com/tatobari/hx711py
# and https://github.com/odriverobotics/ODrive/blob/master/tools/odrive_demo.py

from __future__ import print_function

import odrive
from odrive.enums import *
from odrive.utils import *
import sys
import argparse
import csv
import time
import math
import RPi.GPIO as GPIO
from hx711 import HX711

def main():
    referenceUnit = 22*876/881

    parser = argparse.ArgumentParser(description='Runs ODrive velocity controller under current limiting')
    parser.add_argument("-c", "--comment",\
        help="enter comment string to be included in output csv",
        type=str)
    parser.add_argument("-i", "--current",\
        help="enter current limit in Amps (default = 10A)",
        type=float)
    parser.add_argument("-s", "--speed",\
        help="enter speed in rev/s (i.e., Hz) (default = 2 Hz)",
        type=float)
    parser.add_argument("-f", "--friction",\
        help="enter torque to compensate for system friction (default = 0.050 Nm)",
        type=float)

    args = parser.parse_args()

    hx = HX711(5, 6)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(referenceUnit)
    hx.reset()

    hx.tare()

    print("load cell tare done...")

    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    my_drive = odrive.find_any()
    ax = my_drive.axis0

    # Calibrate motor and wait for it to finish
    print("odrive found. starting calibration...")
    ax.motor.config.current_lim = 10.0
    if args.current is not None:
        ax.motor.config.current_lim = args.current
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    # ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    val = hx.get_weight(1)
    # start_liveplotter(lambda:[ax.encoder.vel_estimate,\
    #     ax.controller.vel_setpoint,\
    #     ax.motor.current_control.Iq_setpoint,\
    #     val/1000.0])
    ax.controller.input_vel = 2.0
    if args.speed is not None:
        ax.controller.input_vel = args.speed
    friction_comp = 0.05
    if args.friction is not None:
        friction_comp = args.friction
    t0 = time.monotonic()
    data = []
    data.append(["# Test started at " + time.asctime()])
    data.append(["# vbus = " + '{}'.format(round(my_drive.vbus_voltage, 3)) + " V"])
    data.append(["# current limit = " + '{}'.format(round(ax.motor.config.current_lim, 3)) + " A"])
    if args.comment is not None:
        data.append(["# user comment: " + args.comment])
    data.append(["time [s]","velocity setpoint [Hz]",\
        "velocity measured [Hz]","motor current [A]",\
        "load cell weight [g]",\
        "motor torque [Nm]",\
        "brake torque [Nm]",\
        "compensated brake torque [Nm]"])
    kt = ax.motor.config.torque_constant
    while True:
        try:        
            weight = hx.get_weight(1)
            # print('weight: {}'.format(weight))
            brake_torque = weight*-0.001*9.81*5*2.54/100
            row = [time.monotonic() - t0,\
                ax.controller.vel_setpoint,\
                ax.encoder.vel_estimate,\
                ax.motor.current_control.Iq_measured,\
                -1.0*weight,\
                ax.motor.current_control.Iq_measured*kt,\
                brake_torque,\
                brake_torque + np.sign(ax.encoder.vel_estimate)*friction_comp]
            data.append(row)

            # time.sleep(0.01)

        except (KeyboardInterrupt, SystemExit):
            print("Cleaning...")
            GPIO.cleanup()
            ax.requested_state = AXIS_STATE_IDLE
            with open("data/" + sys.argv[0][:-3] + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
                csvWriter = csv.writer(my_csv,delimiter=',')
                csvWriter.writerows(data)
            print("Bye!")
            sys.exit()
        
        except :
            ax.requested_state = AXIS_STATE_IDLE

if __name__ == "__main__" :
    main()
