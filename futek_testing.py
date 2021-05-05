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
from utils import *

import Adafruit_ADS1x15

# def adc2temp(temp):
#     R0 = 100000
#     T0 = 25

#     Rf = 100000
#     V0 = 5

#     Vt = round(6.144*(2.0*temp/(65536)), 6)

#     Rt = Rf*Vt / (V0 - Vt)

#     temp = (1/298.15) + (1/3950)*math.log(Rt/R0)
#     return 1/temp - 273.15

# def adc2futek(adc, gain=1, zero_val=2.500):
#     torque = 6.144*(2.0*adc/(65536))
#     torque = -2.0*(torque-zero_val) * gain
#     return torque

async def finish(c1, c2, data=None):
    if data is not None:
        print("writing data...")
        with open("futek_data/futek_test" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            csvWriter.writerows(data)
    
    print("stopping actuators and cleaning...")
    await asyncio.sleep(0.1)
    await c1.set_stop()
    await c2.set_stop()
    await asyncio.sleep(0.1)
    os.system("sudo ip link set can0 down")

    print("done.\n\n")

async def main():
    parser = argparse.ArgumentParser(description='Runs dual-actuator moteus controller futek dynamometer setup')
    parser.add_argument("-c", "--comment",\
        help="enter comment string to be included in output csv. DO NOT INCLUDE ANY COMMAS.",
        type=str, required=True)
    parser.add_argument("-g1", "--gear1",\
        help="specify gear ratio of actuator test sample",
        type=float, required=True)
    parser.add_argument("-g2", "--gear2",\
        help="specify gear ratio of load actuator",
        type=float, required=True)
    parser.add_argument("-d", "--damping",\
        help="load damping scale. unitless",
        type=float)
    parser.add_argument("--duration",\
        help="test duration in seconds",
        type=float)

    input_group = parser.add_mutually_exclusive_group()
    input_group.add_argument("-s", "--step",\
        help="run a step response test. provide step input current to test actuator in A",
        type=float)
    input_group.add_argument("--stair",\
        help="run stairstep input (configured in the file)",action='store_true')
    input_group.add_argument("--grp",\
        help="run a lowpass filtered gaussian random process (configure in file)", action='store_true')

    parser.add_argument("--antihysteresis",\
        help="runs anti-hysteresis pattern",action='store_true')

    parser.add_argument("--torquesensor",\
        help="specify which torque sensor is being used", choices=['trd605-18', 'trs605-5'],\
        type=str, required=True)

    parser.add_argument("--loadmode",\
        help="specify how load actuator behaves", choices=['damping', 'damp', 'stall', 'stop', 'idle'],\
        type=str, required=True)

    parser.add_argument("--drivemode",\
        help="specify how driving actuator behaves", choices=['velocity', 'current'],\
        type=str, required=True)

    speed_group = parser.add_mutually_exclusive_group()
    speed_group.add_argument("--fast",\
        help="skip temp readings, only command c1",action='store_true')
    speed_group.add_argument("--ultrafast",\
        help="skip temp and futek readings, only command c1",action='store_true')

    args = parser.parse_args()

    g1 = args.gear1 if args.gear1 is not None else 6.0
    g2 = args.gear2 if args.gear2 is not None else 6.0
    step_mag = args.step if args.step is not None else 0.0 
    damping = args.damping if args.damping is not None else 0.1 

    ts_gain = 0; ts_overload = 0
    if args.torquesensor == 'trd605-18': ts_gain = 18.0/5.0; ts_overload = 18
    elif args.torquesensor == 'trs605-5': ts_gain = 5.0/5.0; ts_overload = 5

    Ts = 0.01
    GRP = GaussianRandomProcess(mean=0, amplitude=5, Ts=Ts)
    # SET GRP cutoff frequency here
    GRP.set_fc(30)
    
    c1, c2, kt_1, kt_2 = await init_controllers()

    await c1.set_rezero()
    await c2.set_rezero()

    adc = Adafruit_ADS1x15.ADS1115()
    GAIN = 2.0/3.0
    DATARATE = 860

    zero_val = adc.read_adc(1, gain=GAIN, data_rate=DATARATE)
    zero_val = round(6.144*(2.0*zero_val/(65536)), 6)

    if args.fast: adc.start_adc(1, gain=GAIN, data_rate=DATARATE)
    
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
    data.append(["# kt_1 = {}; kt_2 = {} from calibration logs".format(kt_1, kt_2)])
    data.append(["# g1 = {}; g2 = {}".format(g1, g2)])
    data.append(["# torque sensor: {}".format(args.torquesensor)])
    data.append(["# load mode: {}".format(args.loadmode)])
    data.append(["# drive mode: {}".format(args.drivemode)])
    data.append(["# load damping scale = {}".format(damping)])
    if args.step is not None: data.append(["# step input test; magnitude = {} A".format(step_mag)])
    if args.comment is not None: data.append(["# User comment: " + args.comment.replace(',',';')])

    cmd_label = 'q-axis cmd [A]'
    if args.drivemode == 'velocity': cmd_label = 'velocity cmd [Hz]'

    data.append(["time [s]",\
                "time function [s]",\
                "a1 {}".format(cmd_label),\
                "a1 torque cmd [Nm]",\
                "a1 position [rad]", "a1 velocity [rad/s]", "a1 torque [Nm]", "a1 q-axis [A]",\
                "a2 {}".format(cmd_label),\
                "a2 torque cmd [Nm]",\
                "a2 position [rad]", "a2 velocity [rad/s]", "a2 torque [Nm]", "a2 q-axis [A]",\
                "c1 mode", "c1 position [rev]", "c1 vel [Hz]",\
                "c1 torque [Nm]", "c1 voltage [V]",\
                "c1 temp [C]", "c1 fault",\
                "c2 mode", "c2 position [rev]", "c2 vel [Hz]",\
                "c2 torque [Nm]", "c2 voltage [V]",\
                "c2 temp [C]", "c2 fault",\
                "{} torque [Nm]".format(args.torquesensor),
                "motor temp [C]", "housing temp [C]",\
                "observed torque constant [Nm/A]"])

    overtemp = False
    old_cmd = 0.0
    cmd = 0.0
    if not args.fast or args.ultrafast:
        temp1 = adc.read_adc(2, gain=GAIN, data_rate=DATARATE); temp1 = adc2temp(temp1)
        temp2 = adc.read_adc(3, gain=GAIN, data_rate=DATARATE); temp2 = adc2temp(temp2)
    else:
        temp1 = 0; temp2 = 0
    t0_fcn = t0
    ca = c1
    cb = c2
    orient_a_1 = True # True when a1 is driving
    # ca = c2
    # cb = c1
    # orient_a_1 = False # True when a1 is driving
    
    pos_neg = 1 # positive or negative command
    cmd1 = 0; cmd2 = 0
    replya = await ca.set_stop(query=True)
    replyb = await cb.set_stop(query=True)

    # parameters for stairstep command
    max_cmd = 1.1   # A     or rotation Hz in velocity mode
    hold = 1        # s
    incr = 0.25      # A     or rotation Hz in velocity mode
    rate = incr/hold      # A/s   or rotation Hz/s in velocity mode

    cycle = 1
    t_fcn = 0
    t_vec = np.array([])
    t_old = t0
    while True:
        try:
            t = time.monotonic() - t0
            dt_med = t - t_old
            t_old = t
            np.append(t_vec,t)
            frame = min(max(len(t_vec) - 2, 1), 20)
            if frame > 1:
                dt_vec = t_vec[-frame:] - t_vec[-frame-1:-1]
                dt_med = np.median(dt_vec)
            else:
                dt_med = 0.005
            GRP.set_Ts(dt_med)
            # Cycle input function
            if t_fcn > (max_cmd+incr)/rate:
                # Swap driving and loading actuators
                if not args.fast and not args.ultrafast:
                    ctemp = ca
                    ca = cb
                    cb = ctemp
                    orient_a_1 = not orient_a_1 # swap driving actuator
                    print("reverse! reverse!")
                if orient_a_1: pos_neg = -pos_neg # swap cmd sign every other cycles
                t0_fcn = time.monotonic()
                # print("cycle")
                cycle += 1

            if args.duration is not None and t > args.duration:
                print("test duration done")
                await finish(c1, c2, data)
                return
            
            # Setup t_fcn -- freezes when overtemp is latched
            if not overtemp: t_fcn = time.monotonic() - t0_fcn
            
            old_cmd = cmd
            freq_hz = 0
            # Generate Command
            if args.step: cmd = step_mag
            elif args.stair:
                cmd0 = pos_neg * incr*(min(rate*(t_fcn), max_cmd)//incr)

                # modulate command to combat/balance out hysteresis effects
                if args.antihysteresis: 
                    if(  ((rate*t_fcn)%incr)/incr > 0.01 and ((rate*t_fcn)%incr)/incr <= 0.2): cmd = cmd0
                    elif(((rate*t_fcn)%incr)/incr > 0.2 and ((rate*t_fcn)%incr)/incr <= 0.4): cmd = cmd0 + min(0.5*incr, 0.3)
                    elif(((rate*t_fcn)%incr)/incr > 0.4 and ((rate*t_fcn)%incr)/incr <= 0.6): cmd = cmd0
                    elif(((rate*t_fcn)%incr)/incr > 0.6 and ((rate*t_fcn)%incr)/incr <= 0.8): cmd = cmd0 - min(0.5*incr, 0.3)
                    elif(((rate*t_fcn)%incr)/incr > 0.8 and ((rate*t_fcn)%incr)/incr <= 0.9): cmd = cmd0
                    elif(((rate*t_fcn)%incr)/incr > 0.9): cmd = 0
                    else: cmd = 0
                else:
                    cmd = cmd0
                # freq_hz = ((0.5*t)//1.0) # exponent
                # freq_hz = min(1.0*(1.1**freq_hz), 45) # increase freq by 10% every 2 sec
                # cmd = max_cmd*math.cos(freq_hz*np.pi*t)
            elif args.grp:
                cmd = GRP.sample()[0]
            
            # Overtemp Detection and Latch
            if min(temp1, temp2) > 45 or max(temp1, temp2) > 85 or overtemp:
                if t % 1.0 < 0.019 or overtemp == False and not args.fast:
                    print("over temp: temp1 = {}, temp2 = {}".format(round(temp1, 2), round(temp2, 2)))
                # await finish(c1,c2,data)
                # return
                overtemp = True
                cmd = 0.0

            # Overtemp unlatch
            if min(temp1, temp2) < 40 and max(temp1, temp2) < 70 and overtemp:
                overtemp = False

            # Load Actuator
            if not args.fast and not args.ultrafast:
                if args.loadmode == 'damping' or args.loadmode == 'damp':
                    replyb = (await cb.set_position(position=math.nan, velocity=0,\
                        watchdog_timeout=1.0, kp_scale=0.0, kd_scale=damping, query=True))
                elif args.loadmode == 'stall':
                    replyb = (await cb.set_position(position=0.0, velocity=math.nan,\
                        watchdog_timeout=2.0, kp_scale=10, kd_scale=1, query=True))
                else:
                    replyb = await cb.set_stop(query=True)
                
            # Driving Actuator
            if args.drivemode == 'current':
                replya = (await ca.set_current(q_A=cmd, d_A=0.0, query=True))
            elif args.drivemode == 'velocity':
                replya = (await ca.set_position(position=math.nan, velocity=cmd,\
                    watchdog_timeout=2.0, query=True))
            else:
                cmd = 0
                replya = await ca.set_stop(query=True)

            # Keep track of data
            if orient_a_1:
                reply1 = replya
                reply2 = replyb
                cmd1 = cmd
                cmd2 = 0
            else:
                reply2 = replya
                reply1 = replyb
                cmd1 = 0
                cmd2 = cmd

            # Position, velocity, torque
            p1, v1, t1 = parse_reply(reply1, g1)
            p2, v2, t2 = parse_reply(reply2, g2)

            # Read from analog sensors (torque and thermistors)
            if not args.fast and not args.ultrafast:
                futek_torque = adc.read_adc(1, gain=GAIN, data_rate=DATARATE); futek_torque = adc2futek(futek_torque, gain=5/5)
                temp1 = adc.read_adc(2, gain=GAIN, data_rate=DATARATE); temp1 = adc2temp(temp1)
                temp2 = adc.read_adc(3, gain=GAIN, data_rate=DATARATE); temp2 = adc2temp(temp2)
            elif args.fast:
                futek_torque = adc.get_last_result(); futek_torque = adc2futek(futek_torque, gain=5/5)
                temp1 = 0; temp2 = 0
            else:
                futek_torque=0;temp1=0;temp2=0

            if t % 1.0 < 0.02 and not args.fast and not args.ultrafast: print("t = {}s, temp1 = {}, temp2 = {}, cycle = {}, cmd = {}, t1 = {}, t2 = {}".format(\
                round(t, 3), round(temp1, 2), round(temp2, 2), cycle, round(cmd, 4), round(t1, 3), round(t2, 3)))

            observed_kt = 0 if np.abs(cmd) < 0.001 else futek_torque/cmd

            # Log data
            row = [t, t_fcn] + [cmd1*g1 if args.drivemode=='velocity' else cmd1] + [cmd1*kt_1*g1] +\
                [p1, v1, t1, t1/(kt_1*g1)] + [cmd2*g2 if args.drivemode=='velocity' else cmd2] + [cmd2*kt_2*g2] + [p2, v2, t2, t2/(kt_2*g2)]\
                + raw_reply_list(reply1) + raw_reply_list(reply2) +\
                [futek_torque, temp1, temp2, observed_kt]
            data.append(row)

            # Torque overload protection
            if abs(t1) > ts_overload or abs(t2) > ts_overload:
                print("torque overload detected")
                await finish(c1, c2, data)
                # sys.exit()
                return

            if args.fast: await asyncio.sleep(0.001)

        except (KeyboardInterrupt, SystemExit):
            print("\nCaught keyboard interrupt; exiting...")
            await finish(c1, c2, data)
            # sys.exit()
            return
        except:
            print("\n\nUnanticipated exception: something went wrong\n\n")
            await finish(c1, c2, data)
            raise
            # sys.exit()
            return
    
    await finish(c1,c2,data)

if __name__ == "__main__" :
    asyncio.run(main())

