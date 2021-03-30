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

async def main():
    c1, c2, kt_1, kt_2 = await init_controllers()
    print(kt_1)
    print(kt_2)

    p1, v1, t1 = parse_reply(await c1.set_stop(query=True))
    p2, v2, t2 = parse_reply(await c2.set_stop(query=True))

    kin = Kinematics()

    [x,y] = kin.fk_vec(p1,p2)[-1]

    [x0,y0] = [x+20,y-10]

    k = 0.5

    while True:
        try:
            fx = -k*(x-x0)
            fy = -k*(y-y0)

            J = kin.jacobian_theta(p1,p2)
            Jinv = LA.inv(J)
            dt = np.matmul(Jinv, np.array([[fx],[fy]]))
            t1 = dt[0,0]
            t2 = dt[1,0]
            # print('fx={} fy={} t1={} t2={}'.format(fx, fy, t1, t2))
            p1, v1, t1 = parse_reply(await c1.set_current(q_A=t1/kt_1, d_A=0.0, query=True))
            p2, v2, t2 = parse_reply(await c2.set_current(q_A=t2/kt_2, d_A=0.0, query=True))
            print('p1={},v1={},t1={}'.format(p1, v1, t1))
            print('p2={},v2={},t2={}'.format(p2, v2, t2))
            [x,y] = kin.fk_vec(p1,p2)[-1]
            time.sleep(0.005)
        except (KeyboardInterrupt, SystemExit):
            print("stopping actuators and cleaning...")
            await c1.set_stop()
            await c2.set_stop()
            await asyncio.sleep(0.2)
            os.system("sudo ip link set can0 down")
            print("done")
            sys.exit()
        except:
            os.system("sudo ip link set can0 down")
            print("something went wrong")
            raise
            sys.exit()

if __name__ == "__main__" :
    asyncio.run(main())

