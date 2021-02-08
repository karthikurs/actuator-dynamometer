#!/usr/bin/env python3

# Based on example.py from https://github.com/tatobari/hx711py
# and https://github.com/odriverobotics/ODrive/blob/master/tools/odrive_demo.py

from __future__ import print_function

import sys
import argparse
import csv
import time
import math
import RPi.GPIO as GPIO
from hx711 import HX711

def main():
    referenceUnit = 22*876/881

    parser = argparse.ArgumentParser(description='Runs ODrive velocity controller under current limiting')
    parser.add_argument("-c", "--comment",\
        help="enter comment string to be included in output csv",
        type=str)

    args = parser.parse_args()

    hx = HX711(5, 6)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(referenceUnit)
    hx.reset()

    hx.tare()

    print("load cell tare done...")

    val = hx.get_weight(1)
    friction_comp = 0.05
    t0 = time.monotonic()
    data = []
    data.append(["# Test started at " + time.asctime()])
    if args.comment is not None:
        data.append(["# user comment: " + args.comment])
    data.append(["time [s]",\
        "load cell weight [g]",\
        "brake torque [Nm]"])
    while True:
        try:        
            weight = hx.get_weight(1)
            # print('weight: {}'.format(weight))
            brake_torque = weight*-0.001*9.81*5*2.54/100
            row = [time.monotonic() - t0,\
                -1.0*weight,\
                brake_torque]
            data.append(row)

            # time.sleep(0.01)

        except (KeyboardInterrupt, SystemExit):
            print("Cleaning...")
            GPIO.cleanup()
            with open("data/" + sys.argv[0][:-3] + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
                csvWriter = csv.writer(my_csv,delimiter=',')
                csvWriter.writerows(data)
            print("Bye!")
            sys.exit()
        
        except :
            ax.requested_state = AXIS_STATE_IDLE

if __name__ == "__main__" :
    main()
