# EMBiR Modular Leg

This repo contains code to run and test the Modular Leg, as well as its custom actuators

### brake-dynamometer/

Code and data associated with the bike-brake dynamometer (no longer in use)

### figures/

Misc. figures taken during testing

### futek_data/

Actuator data taken during testing with Futek sensor (main data storage)

### moteus-setup/
Misc. files to set up moteus controllers
moteus-setup/moteus-cal/ri50_cal_*.log: moteus calibration files specific to the motors.

### .


`ads_example.py`: runs ads1115/ads1015 ADC and prints to terminal for debugging

`futek_testing.py`: main actuator dynamometer testing script

`grp.py`: Gaussian Random Process generator class used in `futek_testing.py`

`ina260_test.py`: runs ina260 power sensors and prints to terminal for debugging

`kinematics.py`: leg kinematics calculations

`moteus_dual_actuator.py`: sandbox script for testing leg

`moteus_wrapper.py`: wraps moteus library commands

`stop-moteus.py`: sends stop commands to moteus controllers

`utils.py`: python functions for various scripts (mostly futek_testing)

`visualize_data.py`: command-line interactive plotting script


## Reference
This code assumes you are using the mjbots moteus motor drivers and pi3hat:
mjbots/moteus: https://github.com/mjbots/moteus
mjbots/pi3hat: https://github.com/mjbots/pi3hat

Please refer to the documentation in those repos as needed.

### Setting up a new moteus driver
Moteus drivers communicate over CAN-FD, which addresses devices via IDs. By default, new moteus drivers are ID 1. If you need to run multiple simulataneously, the IDs for each must be unique. Follow these step to set an ID:

1. Connect only the moteus driver of interest to the CAN bus.
2. Power up the driver
3. Run `python3 -m moteus_gui.tview --devices=1 --target=1` to open the tview GUI
4. Click on the config tab in the upper left
5. Expand the ID folder
6. Double click the value next to ID
7. Type in your desired ID. This will immediately change the ID, so this tview window will no longer be valid. Close the window.
8. Run `python3 -m moteus_gui.tview --devices=1 --target=<ID>` where `<ID>` is the new ID you set.
9. Run `conf write` in the tview terminal to make the ID change persistent.

### Running moteus GUI (tview) with multiple drives connected
Mjbots provides a command and telemetry GUI which is helpful for quick verification testing and debugging. Run:
`sudo python3 -m moteus_gui.tview --pi3hat-cfg '1=1,2' -t 1,2`
Here, we assume targets IDs 1 and 2 exist and are connected to bus 1 on the pi3hat. If that is not the configuration you are running, please refer to the pi3hat documentation on how to construct the arguments correctly.

Refer to the moteus documentation on the tview command line commands available: https://github.com/mjbots/moteus/blob/main/docs/reference.md#b-diagnostic-command-set

### Motor Calibration
Motor calibration is necessary for the moteus drivers to determine parameters necessary for motor control. In order to run calibration:
1. Ensure motors are free to rotate (frictional load ok)
2. Power up main power
3. Run: `python3 -m moteus.moteus_tool --target <ID> --calibrate` where `<ID>` should be replaced with the moteus driver IDs. Use 1 and 2.
4. The command above should generate a .log file in the cwd. Rename the file to `ri50_cal_<ID>.log` and move it to `moteus-setup/moteus-cal/`
5. If desired, you can manually change the `v_per_hz` parameter in these log files to set a different torque constant based on a priori knowledge. `kt = 0.78*v_per_hz/pi`.

To restore a calibration log file, run `python3 -m moteus.moteus_tool --target <ID> --restore-cal <cal_file>`

### Sample Python script:

https://github.com/mjbots/moteus/blob/main/lib/python/examples/pi3hat_multiservo.py

You can also use the functions in `moteus_wrapper.py`, using the controller objects returned by the wrapper functions rather than the ones constructed in the sample script.
