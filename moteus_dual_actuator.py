#!/usr/bin/env python3

from __future__ import print_function

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

import Adafruit_ADS1x15


async def main():
    print("bringing up CAN...")
    os.system("sudo ip link set can0 down")
    os.system("sudo ip link set can0 up type can   tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5   dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3   restart-ms 1000 fd on")
    c1 = moteus.Controller(id=1)
    c2 = moteus.Controller(id=2)
    
    stream1 = moteus.Stream(c1)
    stream2 = moteus.Stream(c2)
    
    cal_file_1 = "moteus-setup/moteus-cal/ri50_cal_1.log"
    cal_file_2 = "moteus-setup/moteus-cal/ri50_cal_2.log"
    
    print("loading moteus controller calibration from " + cal_file_1 + ", " +cal_file_2 + " ...")
    cmd1 = "python3 -m moteus.moteus_tool --target 1 --restore-cal " + cal_file_1
    cmd2 = "python3 -m moteus.moteus_tool --target 2 --restore-cal " + cal_file_2
    print(cmd1)        
    os.system(cmd1)
    print(cmd2)
    os.system(cmd2)
    
    cal1 = json.load(open(cal_file_1, "r"))
    kt_1 = 30/(math.pi*cal1["kv"])
    cal2 = json.load(open(cal_file_2, "r")) 
    kt_2 = 30/(math.pi*cal2["kv"])
    
    await c1.set_stop()
    await c2.set_stop()
    await c1.set_rezero()
    await c2.set_rezero()
    kp = 0.3
    kd = 0.5

    adc = Adafruit_ADS1x15.ADS1115()
    GAIN = 2.0/3.0

    zero_val = adc.read_adc(1, gain=GAIN)
    zero_val = round(6.144*(2.0*zero_val/(65536)), 6)
    print(zero_val)
    
    t0 = time.monotonic()
    data = []
    data.append(["# Test started at " + time.asctime()])
    data.append(["time [s]",
                "c1 torque cmd [Nm]",
                "c1 mode", "c1 position [rev]", "c1 vel [Hz]",\
                "c1 torque [Nm]", "c1 voltage [V]",\
                "c1 temp [C]", "c1 fault",\
                "c2 mode", "c2 position [rev]", "c2 vel [Hz]",\
                "c2 torque [Nm]", "c2 voltage [V]",\
                "c2 temp [C]", "c2 fault",\
                "trd605 torque [Nm]"])
    
    pos = 0.5
    while True:
        try:
            # reply1 = (await c1.set_position(position=pos, watchdog_timeout=2.0, kp_scale=kp, kd_scale=kd, query=True))
            # reply2 = (await c2.set_position(position=-2*pos, watchdog_timeout=2.0, kp_scale=kp, kd_scale=kd, query=True))
            t = time.monotonic() - t0
            # cmd = 4.0*math.sin(t)
            cmd = min(0.5*t, 10)
            reply1 = (await c1.set_current(q_A=cmd, d_A=0.0, query=True))
            # reply2 = (await c2.set_current(q_A=0.0, d_A=0.0, query=True))
            reply2 = (await c2.set_position(position=math.nan, velocity=0.0,\
                watchdog_timeout=2.0, kp_scale=0, kd_scale=3.0, query=True))
            pos = -pos

            # print(reply1)
            # print(reply2)

            futek_torque = adc.read_adc(1, gain=GAIN)
            futek_torque = round(6.144*(2.0*futek_torque/(65536)), 6)
            futek_torque = -2.0*(futek_torque-zero_val) * 18.0/5.0
            row = [t] + [cmd*kt_1] + [val for key, val in reply1.values.items()] + [val for key, val in reply2.values.items()] + [futek_torque]
            # print(futek_torque)
            # print(reply1.values[1])
            # break
            data.append(row)
            # time.sleep(1.5)
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
            sys.exit()
        
        except :
            print("something went wrong")
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
            sys.exit()
    
    print("stopping actuators and cleaning...")
    await c1.set_stop()
    await c2.set_stop()
    await asyncio.sleep(0.2)
    os.system("sudo ip link set can0 down")
    print("done")
    sys.exit()

if __name__ == "__main__" :
    asyncio.run(main())