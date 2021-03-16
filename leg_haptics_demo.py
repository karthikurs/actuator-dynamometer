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
from .moteus_wrapper import *
from .kinematics import *

async def main():
    c1, c2 = init_controllers()

    p1, v1, t1 = parse_reply(await c1.set_stop(query=True))
    p2, v2, t2 = parse_reply(await c2.set_stop(query=True))

    kin = Kinematics()

    [x,y] = kin.fk_vec(p1,p2)[-1]

    [x0,y0] = [x+20,y-10]

    k = 0.1

    while True:
        try:
            fx = -k*(x-x0)
            fy = -k*(y-y0)

            J = kin.jacobian_theta(p1,p2)
            Jinv = LA.inv(J)
            dt = np.matmul(Jinv, np.array([[fx],[fy]]))
            t1 = dt[0,0]
            t2 = dt[1,0]
            p1, v1, t1 = parse_reply(await c1.set_current(q_A=t1, d_A=0.0, query=True))
            p2, v2, t2 = parse_reply(await c2.set_current(q_A=t1, d_A=0.0, query=True))
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

