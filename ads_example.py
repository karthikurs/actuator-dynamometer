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

print('Reading ADS1x15 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
print('-' * 37)
# Main loop.
while True:
    # Read all the ADC channel values in a list.
    values = [0]*4
    for i in range(4):
        # Read the specified ADC channel using the previously set gain value.
        values[i] = adc.read_adc(i, gain=GAIN, data_rate=3300)
        # values[i] = round(6.144*(2.0*values[i]/(65536)), 3)
        # if i == 2 or i == 3: values[i] = adc2temp(values[i])
        # elif i == 0: 
            # values[i] = round(4.096/GAIN*(2.0*values[i]/(65536)), 6)s
            # values[i] = -2.0*(values[i]-2.500) * 18.0/5.0
            # values[i] = -2.0*(values[i]- (3.3/2)) * 5.0/3.3
            # values[i] = adc2futek(values[i], gain=5/5)
            # values[i] = 4.096*2*values[i]/4096.0

        # Note you can also pass in an optional data_rate parameter that controls
        # the ADC conversion time (in samples/second). Each chip has a different
        # set of allowed data rate values, see datasheet Table 9 config register
        # DR bit values.
        #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        # Each value will be a 12 or 16 bit signed integer value depending on the
        # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
    # Print the ADC values.
    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
    # Pause for half a second.
    time.sleep(0.5)
