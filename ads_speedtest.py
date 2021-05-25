# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
import time
import math
from utils import *

# Import the ADS1x15 module.
import Adafruit_ADS1x15

# def adc2temp(temp, V0 = 3.3):
#     R0 = 100000
#     T0 = 25

#     Rf = 100000
#     # V0 = 5

#     Vt = round(6.144*(2.0*temp/(65536)), 6)
#     # Vt = round(4.096*(2.0*temp/(65536)), 6)

#     Rt = Rf*Vt / (V0 - Vt)

#     temp = (1/298.15) + (1/3950)*math.log(Rt/R0)
#     return 1/temp - 273.15

# Create an ADS1115 ADC (16-bit) instance.
# adc = Adafruit_ADS1x15.ADS1115()

# Or create an ADS1015 ADC (12-bit) instance.
adc = Adafruit_ADS1x15.ADS1015()

# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
# GAIN = 2.0/3.0
GAIN = 1.0
DATARATE = 3300
n = 100
adc.start_adc(0, gain=GAIN, data_rate=DATARATE)
# Main loop
t0 = time.monotonic()
for ii in range(n):
    futek_torque = adc.get_last_result()
tspan = time.monotonic() - t0
print("read {} samples in {} s; Ts = {}; fs = {}".format(\
    n, round(tspan, 3), round(tspan/n, 5), round(n/tspan, 3)))