#!/usr/bin/env python3

from __future__ import print_function

import moteus
import moteus.moteus_tool as mt

import numpy as np
from numpy import sin, cos
from numpy import linalg as LA
from numpy import linspace

import asyncio
import sys
import os
import argparse
import csv
import json
import time
import math
from moteus_wrapper import *
from kinematics import *

import Adafruit_ADS1x15


async def main():
    parser = argparse.ArgumentParser(description='Runs dual-actuator moteus controller futek dynamometer setup')
    parser.add_argument("-c", "--comment",\
        help="enter comment string to be included in output csv",
        type=str)

    args = parser.parse_args()
    
    c1, c2, kt_1, kt_2 = await init_controllers()

    p1, v1, t1 = parse_reply(await c1.set_stop(query=True))
    p2, v2, t2 = parse_reply(await c2.set_stop(query=True))
    await c1.set_rezero()
    await c2.set_rezero()

    adc = Adafruit_ADS1x15.ADS1115()
    GAIN = 2.0/3.0

    zero_val = adc.read_adc(1, gain=GAIN)
    zero_val = round(6.144*(2.0*zero_val/(65536)), 6)
    print(zero_val)
    
    t0 = time.monotonic()
    data = []
    data.append(["# Test started at " + time.asctime()])
    data.append(["# Labels:"])
    data.append(["# \t a1 = actuator 1. Values at *actuator* level (6:1 gearbox)"])
    data.append(["# \t a2 = actuator 2. Values at *actuator* level (6:1 gearbox)"])
    data.append(["# \t c1 = moteus controller 1. Values at *motor* level (no gearbox)"])
    data.append(["# \t c2 = moteus controller 2. Values at *motor* level (no gearbox)"])
    data.append(["# Note: Data assumes gearbox is 100pc efficient"])
    data.append(["# "])
    data.append(['# kt_1 = {}, kt_2 = {}'.format(kt_1, kt_2)])
    if args.comment is not None:
        data.append(["# User comment: " + args.comment])

    data.append(["time [s]",
                "a1 q-axis cmd [A]",
                "a1 torque cmd [Nm]",
                "a1 position [rad]", "[a1 velocity [rad/s]", "a1 torque [Nm]",
                "a2 position [rad]", "[a2 velocity [rad/s]", "a2 torque [Nm]",
                "c1 mode", "c1 position [rev]", "c1 vel [Hz]",\
                "c1 torque [Nm]", "c1 voltage [V]",\
                "c1 temp [C]", "c1 fault",\
                "c2 mode", "c2 position [rev]", "c2 vel [Hz]",\
                "c2 torque [Nm]", "c2 voltage [V]",\
                "c2 temp [C]", "c2 fault",\
                "trd605 torque [Nm]"])

    while True:
        try:
            t = time.monotonic() - t0
            
            # cmd = 4.0*math.sin(t)
            cmd = min(0.5*t, 8)//1.0

            reply1 = (await c1.set_current(q_A=cmd, d_A=0.0, query=True))
            # reply2 = (await c2.set_current(q_A=0.0, d_A=0.0, query=True))
            reply2 = (await c2.set_position(position=math.nan, velocity=0.0,\
                watchdog_timeout=2.0, kp_scale=0, kd_scale=1.5, query=True))

            p1, v1, t1 = parse_reply(reply1)
            p2, v2, t2 = parse_reply(reply2)

            futek_torque = adc.read_adc(1, gain=GAIN)
            futek_torque = round(6.144*(2.0*futek_torque/(65536)), 6)
            futek_torque = -2.0*(futek_torque-zero_val) * 18.0/5.0
            row = [t] + [cmd] + [cmd*kt_1*6] + [p1, v1, t1, p2, v2, t2] + raw_reply_list(reply1) + raw_reply_list(reply2) + [futek_torque]
            data.append(row)
        except (KeyboardInterrupt, SystemExit):
            print("stopping actuators and cleaning...")
            with open("futek_data/futek_test_" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
                csvWriter = csv.writer(my_csv,delimiter=',')
                csvWriter.writerows(data)
            
            await asyncio.sleep(0.1)
            await c1.set_stop()
            await c2.set_stop()
            await asyncio.sleep(0.1)
            os.system("sudo ip link set can0 down")

            print("done")
            # sys.exit()
            return
        except:
            print("something went wrong")
            print("stopping actuators and cleaning...")
            with open("futek_data/futek_test" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
                csvWriter = csv.writer(my_csv,delimiter=',')
                csvWriter.writerows(data)
            
            await asyncio.sleep(0.1)
            await c1.set_stop()
            await c2.set_stop()
            await asyncio.sleep(0.1)
            os.system("sudo ip link set can0 down")

            print("done")
            # sys.exit()
            return

if __name__ == "__main__" :
    asyncio.run(main())

