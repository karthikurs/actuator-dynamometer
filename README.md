# EMBiR Modular Leg

This repo contains code to run and test the Modular Leg, as well as its custom actuators

## brake-dynamometer/

Code and data associated with the bike-brake dynamometer (no longer in use)

## figures/

Misc. figures taken during testing

## futek_data/

Actuator data taken during testing with Futek sensor (main data storage)

## moteus-setup/
Misc. files to set up moteus controllers

## .


`ads_example.py`: runs ads1115/ads1015 ADC and prints to terminal for debugging

`futek_testing.py`: main actuator dynamometer testing script

`grp.py`: Gaussian Random Process generator class used in `futek_testing.py`

`ina260_test.py`: runs ina260 power sensors and prints to terminal for debugging

`kinematics.py`: leg kinematics calculations

`moteus_dual_actuator.py`: sandbox script for testing leg

`moteus_warpper.py`: wraps moteus library commands

`stop-moteus.py`: sends stop commands to moteus controllers

`utils.py`: python functions for various scripts (mostly futek_testing)

`visualize_data.py`: command-line interactive plotting script