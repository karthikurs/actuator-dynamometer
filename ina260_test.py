from ina260.controller import Controller
import time

ina1 = Controller(address=0x40)
ina2 = Controller(address=0x41)

while(True):
	t = time.monotonic()
	print("ina1 v = {},\ti = {},\tp = {}".format(round(ina1.voltage(), 3), round(ina1.current(), 3), round(ina1.power(), 3)))
	print("ina2 v = {},\ti = {},\tp = {}".format(round(ina2.voltage(), 3), round(ina2.current(), 3), round(ina2.power(), 3)))
	print("time = {}\n".format(round(time.monotonic() - t, 4)))
	time.sleep(1)
