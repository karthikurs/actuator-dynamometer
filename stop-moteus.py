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

if __name__ == "__main__" :
    asyncio.run(main())

