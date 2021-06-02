ads_test : ads_test.o Adafruit_ADS1X15.o Adafruit_INA260.o I2CDevice.o
	g++ -ggdb -o ads_test ads_test.o Adafruit_ADS1X15.o Adafruit_INA260.o I2CDevice.o -lbcm2835

Adafruit_ADS1X15.o : Adafruit_ADS1X15.h Adafruit_ADS1X15.cpp I2CDevice.o
	g++ -ggdb -c -pthread Adafruit_ADS1X15.cpp

Adafruit_INA260.o : Adafruit_INA260.h Adafruit_INA260.cpp I2CDevice.o
	g++ -ggdb -c -pthread Adafruit_INA260.cpp

I2CDevice.o : I2CDevice.h I2CDevice.cpp
	g++ -ggdb -c -pthread I2CDevice.cpp -fpermissive