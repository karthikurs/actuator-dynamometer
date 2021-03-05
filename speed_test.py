#!/usr/bin/env python3

# Based on example.py from https://github.com/tatobari/hx711py
# and https://github.com/odriverobotics/ODrive/blob/master/tools/odrive_demo.py

from __future__ import print_function

import odrive
from odrive.enums import *
from odrive.utils import *

import moteus
import moteus.moteus_tool as mt

import asyncio
import sys
import os
import argparse
import csv
import json
import time
import math
import RPi.GPIO as GPIO
from hx711 import HX711

async def main():
    referenceUnit = 22*876/881

    parser = argparse.ArgumentParser(description='Runs motor velocity controller under current limiting')
    parser.add_argument("controller",\
        help="enter controller being used in {odrive, moteus}")
    parser.add_argument("-c", "--comment",\
        help="enter comment string to be included in output csv",
        type=str)
    parser.add_argument("-i", "--current",\
        help="enter current limit in Amps (default = 10A)",
        type=float)
    parser.add_argument("-s", "--speed",\
        help="enter speed in rev/s (i.e., Hz) (default = 2 Hz)",
        type=float)
    parser.add_argument("-d", "--duration",\
        help="enter duration in seconds",
        type=float)
    parser.add_argument("-f", "--friction",\
        help="enter torque to compensate for system friction (default = 0.050 Nm)",
        type=float)
    parser.add_argument("-p", "--profile",\
        help="enter path to .csv dictating speed profile in <speed (Hz), duration (s)> format.\nOverrides -s command.",
        type=str)

    args = parser.parse_args()

    if args.controller is None or (args.controller != 'odrive' and args.controller != 'moteus'):
        print("Invalid controller specified: \"{}\"! Specify either \"odrive\" or \"moteus\" (without quotes)".format(args.controller))
        return

    hx = HX711(5, 6)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(referenceUnit)
    hx.reset()
    hx.tare()
    print("load cell tare done...")

    c = 0
    stream = 0
    ax = 0
    cal = 0
    kt = 0
    if args.controller == 'odrive':
        # Find a connected ODrive (this will block until you connect one)
        print("finding an odrive...")
        my_drive = odrive.find_any()
        ax = my_drive.axis0

        # Calibrate motor and wait for it to finish
        print("odrive found. starting calibration...")
    else:
        os.system("sudo ip link set can0 up type can   tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5   dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3   restart-ms 1000 fd on")
        c = moteus.Controller()
        stream = moteus.Stream(c)
        cal_file = "moteus_testing/moteus_logs/ri50_cal.log"
        print("loading moteus controller calibration from " + cal_file)
        cmd = "python3 -m moteus.moteus_tool --target 1 --restore-cal " + cal_file
        print(cmd)
        os.system(cmd)
        cal = json.load(open(cal_file, "r"))
        kt = 30/(math.pi*cal["kv"])
        await c.set_stop()

    profile = []
    if args.duration is not None:
        profile = pd.DataFrame({'speed':[args.speed],'duration':[args.duration]})
    if args.profile is not None:
        profile = pd.read_csv(args.profile, comment='#', header=0)
    
    if args.controller == 'odrive':
        ax.motor.config.current_lim = args.current if args.current is not None else 10.0
        ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while ax.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        ax.controller.input_vel = args.speed if args.speed is not none else 2.0
        kt = ax.motor.config.torque_constant
    
    val = hx.get_weight(1)
    friction_comp = args.friction if args.friction is not None else 0.05
    
    t0 = time.monotonic()
    data = []
    data.append(["# Test started at " + time.asctime()])
    voltage = my_drive.vbus_voltage if args.controller == 'odrive' else 0.0
    data.append(["# vbus = " + '{}'.format(round(voltage, 3)) + " V"])
    current_lim = ax.motor.config.current_lim if args.controller == 'odrive' else 0
    data.append(["# current limit = " + '{}'.format(round(current_lim, 3)) + " A"])
    if args.comment is not None:
        data.append(["# user comment: " + args.comment])
    data.append(["time [s]","velocity setpoint [Hz]",\
        "velocity measured [Hz]",\
        "motor current measured [A]",\
        "motor current setpoint [A]",\
        "load cell weight [g]",\
        "motor torque measured [Nm]",\
        "motor torque setpoint [Nm]",\
        "brake torque [Nm]",\
        "compensated brake torque [Nm]"])
    i = 1
    while True:
        try:        
            weight = hx.get_weight(1)
            # print('weight: {}'.format(weight))
            brake_torque = weight*-0.001*9.81*5*2.54/100
            row = []
            if args.controller == 'odrive':
                row = [time.monotonic() - t0,\
                    ax.controller.vel_setpoint,\
                    ax.encoder.vel_estimate,\
                    ax.motor.current_control.Iq_measured,\
                    ax.motor.current_control.Iq_setpoint,\
                    -1.0*weight,\
                    ax.motor.current_control.Iq_measured*kt,\
                    ax.motor.current_control.Iq_setpoint*kt,\
                    brake_torque,\
                    brake_torque + np.sign(ax.encoder.vel_estimate)*friction_comp]
                data.append(row)
            else:
                pos = 1.0*i
                print("test")
                reply = (await c.set_position(position=math.nan, velocity=pos,\
                    kp_scale = 2, kd_scale = 0.5,\
                    watchdog_timeout=1.1, query=True))
                # print(reply["VOLTAGE(0X00d)"])
                print(reply)
                # print(reply.values[13])

                q = await c.diagnostic_read()
                print(q)
                i = -i
                await asyncio.sleep(1.0)

            # time.sleep(0.01)

        except (KeyboardInterrupt, SystemExit):
            print("Cleaning...")
            GPIO.cleanup()
            if args.controller == 'odrive':
                ax.requested_state = AXIS_STATE_IDLE
            else:
                await c.set_stop()
                await asyncio.sleep(0.2)
                os.system("sudo ip link set can0 down")
            with open("data/" + sys.argv[0][:-3] + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
                csvWriter = csv.writer(my_csv,delimiter=',')
                csvWriter.writerows(data)
            print("Bye!")
            sys.exit()
        
        except :
            os.system("sudo ip link set can0 down")
            if args.controller == 'odrive':
                ax.requested_state = AXIS_STATE_IDLE

if __name__ == "__main__" :
    asyncio.run(main())
