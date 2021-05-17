from ina260.controller import Controller
import time

ina1 = Controller(address= 0x40)
ina2 = Controller(address= 0x41)

while(True):
	t = time.monotonic()
	v1 = ina1.voltage(); i1 = ina1.current()
	v2 = ina2.voltage(); i2 = ina2.current()
	print("ina1 v = {},\ti = {},\tp = {}".format(round(v1, 3), round(i1, 3), round(v1*i1, 3)))
	print("ina2 v = {},\ti = {},\tp = {}".format(round(v2, 3), round(i2, 3), round(v2*i2, 3)))
	print("time = {}\n".format(round(time.monotonic() - t, 4)))
	time.sleep(0.2)
