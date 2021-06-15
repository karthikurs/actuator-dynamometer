#!/usr/bin/env python3

from __future__ import print_function

import moteus
import moteus.moteus_tool as mt
import moteus_pi3hat
import asyncio
from moteus_wrapper import *

async def main():
    await rezero()
    return

if __name__ == "__main__" :
    asyncio.run(main())

