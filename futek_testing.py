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

def adc2temp(temp):
    R0 = 100000
    T0 = 25

    Rf = 100000
    V0 = 5

    Vt = round(6.144*(2.0*temp/(65536)), 6)

    Rt = Rf*Vt / (V0 - Vt)

    temp = (1/298.15) + (1/3950)*math.log(Rt/R0)
    return 1/temp - 273.15

async def finish(c1, c2, data=None):
    if data is not None:
        print("\nwriting data...")
        with open("futek_data/futek_test" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            csvWriter.writerows(data)
    
    print("\nstopping actuators and cleaning...")
    await asyncio.sleep(0.1)
    await c1.set_stop()
    await c2.set_stop()
    await asyncio.sleep(0.1)
    os.system("sudo ip link set can0 down")

    print("\ndone\n\n")

async def main():
    parser = argparse.ArgumentParser(description='Runs dual-actuator moteus controller futek dynamometer setup')
    parser.add_argument("-c", "--comment",\
        help="enter comment string to be included in output csv. DO NOT INCLUDE ANY COMMAS.",
        type=str)
    parser.add_argument("-g1", "--gear1",\
        help="specify gear ratio of actuator test sample (default = 1.0)",
        type=float)
    parser.add_argument("-g2", "--gear2",\
        help="specify gear ratio of load actuator (default = 1.0)",
        type=float)
    parser.add_argument("-s", "--step",\
        help="run a step response test. provide step input current to test actuator in A",
        type=float)
    parser.add_argument("-d", "--damping",\
        help="load damping scale. unitless",
        type=float)
    parser.add_argument("--duration",\
        help="test duration in seconds",
        type=float)

    args = parser.parse_args()

    g1 = args.gear1 if args.gear1 is not None else 1.0
    g2 = args.gear2 if args.gear2 is not None else 1.0
    step_mag = args.step if args.step is not None else 0.0 
    damping = args.damping if args.damping is not None else 0.1 
    
    c1, c2, kt_1, kt_2 = await init_controllers()

    n = 100
    # tstart = time.monotonic()

    # for _ in range(n):
    #     # p1, v1, t1 = parse_reply(await c1.set_stop(query=True), g1)
    #     # p2, v2, t2 = parse_reply(await c2.set_stop(query=True), g2)
    #     await c1._get_transport().cycle([c.make_stop(query=True) for c in [c1, c2]])
    # tend = time.monotonic()
    # print("cycle command avg time: {}".format((tend-tstart)/n))
    

    await c1.set_rezero()
    await c2.set_rezero()

    adc = Adafruit_ADS1x15.ADS1115()
    GAIN = 2.0/3.0
    DATARATE = 860

    zero_val = adc.read_adc(1, gain=GAIN, data_rate=DATARATE)
    zero_val = round(6.144*(2.0*zero_val/(65536)), 6)
    
    # tstart = time.monotonic()
    # for _ in range(n):
    #     v = adc.read_adc(1, gain=GAIN, data_rate=DATARATE)
    # tend = time.monotonic()
    # print("adc read avg time: {}".format((tend-tstart)/n))

    # await finish(c1, c2)
    # return

    print(zero_val)
    
    t0 = time.monotonic()
    data = []
    data.append(["# Test started at " + time.asctime()])
    data.append(["# Labels:"])
    data.append(["# \t a1 = actuator 1. Values at *actuator* level after gearbox"])
    data.append(["# \t a2 = actuator 2. Values at *actuator* level after gearbox"])
    data.append(["# \t c1 = moteus controller 1. Values at *motor* level (no gearbox)"])
    data.append(["# \t c2 = moteus controller 2. Values at *motor* level (no gearbox)"])
    data.append(["# Note: Data assumes gearbox is 100pc efficient"])
    data.append(["# "])
    data.append(["# kt_1 = {}; kt_2 = {}".format(kt_1, kt_2)])
    data.append(["# load damping scale = {}".format(damping)])
    if args.step is not None: data.append(["# step input test; magnitude = {} A".format(step_mag)])
    if args.comment is not None: data.append(["# User comment: " + args.comment])

    data.append(["time [s]",
                "a1 q-axis cmd [A]",
                "a1 torque cmd [Nm]",
                "a1 position [rad]", "a1 velocity [rad/s]", "a1 torque [Nm]",
                "a2 position [rad]", "a2 velocity [rad/s]", "a2 torque [Nm]",
                "c1 mode", "c1 position [rev]", "c1 vel [Hz]",\
                "c1 torque [Nm]", "c1 voltage [V]",\
                "c1 temp [C]", "c1 fault",\
                "c2 mode", "c2 position [rev]", "c2 vel [Hz]",\
                "c2 torque [Nm]", "c2 voltage [V]",\
                "c2 temp [C]", "c2 fault",\
                "trd605 torque [Nm]",
                "temp 1 [C]", "temp 2 [C]",\
                "observed torque constant [Nm/A]"])

    overtemp = False
    old_cmd = 0.0
    cmd = 0.0
    temp1 = adc.read_adc(2, gain=GAIN, data_rate=DATARATE); temp1 = adc2temp(temp1)
    temp2 = adc.read_adc(3, gain=GAIN, data_rate=DATARATE); temp2 = adc2temp(temp2)
    t0_fcn = t0
    while True:
        try:
            t = time.monotonic() - t0
            t_fcn = time.monotonic() - t0_fcn

            if args.duration is not None and t > args.duration:
                print("test duration done")
                await finish(c1, c2, data)
                return
            
            # cmd = 4.0*math.sin(t)
            max_cmd = 4.0 # A
            rate = 0.5 # A/s
            incr = 0.5 # A
            old_cmd = cmd
            freq_hz = 0
            if args.step is not None: cmd = step_mag if t > 0.0 else 0.0
            else:
                # cmd = incr*(min(rate*(t%20), max_cmd)//incr)
                freq_hz = ((0.5*t)//1.0) # exponent
                freq_hz = min(1.0*(1.1**freq_hz), 45) # increase freq by 10% every 2 sec
                cmd = max_cmd*math.cos(freq_hz*np.pi*t)
            
            # if (t//2.0) % 2 > 0: cmd = 0

            if min(temp1, temp2) > 45 or max(temp1, temp2) > 80 or overtemp:
                if t % 1.0 < 0.012 or overtemp == False:
                    print("over temp: temp1 = {}, temp2 = {}".format(round(temp1, 2), round(temp2, 2)))
                # await finish(c1,c2,data)
                # return
                overtemp = True
                cmd = 0.0

            if min(temp1, temp2) < 35 and max(temp1, temp2) < 70 and overtemp:
                overtemp = False
                t0_fcn = time.monotonic()

            # if cmd != old_cmd: print("cmd = {} A".format(cmd))

            reply1 = (await c1.set_current(q_A=cmd, d_A=0.0, query=True))
            reply2 = (await c2.set_current(q_A=0.0, d_A=0.0, query=True))
            # reply2 = (await c2.set_position(position=math.nan, velocity=0.0,\
            #     watchdog_timeout=2.0, kp_scale=0, kd_scale=damping, query=True))

            p1, v1, t1 = parse_reply(reply1, g1)
            p2, v2, t2 = parse_reply(reply2, g2)

            futek_torque = adc.read_adc(1, gain=GAIN, data_rate=DATARATE)
            futek_torque = round(6.144*(2.0*futek_torque/(65536)), 6)
            futek_torque = -2.0*(futek_torque-zero_val) * 18.0/5.0

            temp1 = adc.read_adc(2, gain=GAIN, data_rate=DATARATE); temp1 = adc2temp(temp1)
            temp2 = adc.read_adc(3, gain=GAIN, data_rate=DATARATE); temp2 = adc2temp(temp2)

            if t % 1.0 < 0.012: print("t = {}s, temp1 = {}, temp2 = {}, freq_hz = {}".format(\
                round(t, 3), round(temp1, 2), round(temp2, 2), round(freq_hz, 2)))

            observed_kt = 0 if np.abs(cmd) < 0.001 else futek_torque/cmd

            row = [t] + [cmd] + [cmd*kt_1*g1] +\
                [p1, v1, t1, p2, v2, t2]\
                + raw_reply_list(reply1) + raw_reply_list(reply2) +\
                [futek_torque, temp1, temp2, observed_kt]
            data.append(row)
        except (KeyboardInterrupt, SystemExit):
            await finish(c1, c2, data)
            # sys.exit()
            return
        except:
            print("\n\nsomething went wrong\n\n")
            await finish(c1, c2, data)
            raise
            # sys.exit()
            return
    
    await finish(c1,c2,data)

if __name__ == "__main__" :
    asyncio.run(main())

