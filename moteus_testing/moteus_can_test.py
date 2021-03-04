#!/usr/bin/python3 -B

import asyncio
import math
import moteus
import moteus.moteus_tool as mt
import argparse
import os

async def main():
    c = moteus.Controller()
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--calibrate", action='store_true')
    # parser.add_argument(
    #     '-t', '--target', type=str, action='append', default=[],
    #     help='destination address(es) (default: autodiscover)')
    # args = parser.parse_args(['--target','1','--calibrate'])
    # mtr = mt.Runner(args)
    # await mtr.start()
    # os.system("python3 -m moteus.moteus_tool --target 1 --calibrate")
    os.system("python3 -m moteus.moteus_tool --target 1 --restore-cal moteus_logs/ri50_cal.log")
    await c.set_stop()  # in case there was a fault
    i = 1
    while True:
        # print(await c.set_position(position=math.nan, velocity=float(i%2), query=True))
        pos = 2.0*i
        # print(await c.set_position(position=pos,\
        #     kp_scale = 2, kd_scale = 0.5,\
        #     watchdog_timeout=math.nan, query=True))
        print(await c.set_position(position=math.nan, velocity=pos,\
            kp_scale = 2, kd_scale = 0.5,\
            watchdog_timeout=math.nan, query=True))
        print(pos)
        # print(await c.query())
        # print(c.id)
        # return
        await asyncio.sleep(2.0)
        i = -i

asyncio.run(main())
