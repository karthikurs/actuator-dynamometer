#include <iostream>
#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"


int main() {
  Adafruit_ADS1015 ads;
  Adafruit_INA260 ina1;
  Adafruit_INA260 ina2;
  uint16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;
  if (!bcm2835_init()) {
    std::cout << "bcm2835_init failed. Are you running as root??\n" << std::endl;
    return 1;
  }
  bcm2835_i2c_begin();
  ads.begin(0x48);
  ads.setGain(adsGain_t::GAIN_ONE);
  ads.setDataRate(RATE_ADS1015_3300SPS);
  std::cout << "set ads params" << std::endl;

  ina1.begin(0x40);
  ina1.prime_i2c();
  ina1.setCurrentConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
  ina1.setVoltageConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
  ina2.begin(0x41);
  ina2.prime_i2c();
  ina2.setCurrentConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
  ina2.setVoltageConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
  for(size_t ii = 0; ii < 10; ++ii) {
    ads.prime_i2c();
    adc0 = ads.readADC_SingleEnded(0);
    // adc1 = ads.readADC_SingleEnded(1);
    // adc2 = ads.readADC_SingleEnded(2);
    // adc3 = ads.readADC_SingleEnded(3);

    volts0 = ads.computeVolts(adc0);
    // volts1 = ads.computeVolts(adc1);
    // volts2 = ads.computeVolts(adc2);
    // volts3 = ads.computeVolts(adc3);
    // std::cout << volts0 << ", " << volts1 << ", " << volts2 << ", " << volts3 << "\n";
    std::cout << volts0 << "\n";
    // std::cout << adc0 << ", " << adc1 << ", " << adc2 << ", " << adc3 << std::endl;
    ina1.prime_i2c();
    std::cout << "ina1: V = " << ina1.readBusVoltage()/1000 << ", I = " << ina1.readCurrent()/1000 << "\n";
    // ina2.prime_i2c();
    // std::cerr << "ina2: V = " << ina2.readBusVoltage()/1000 << ", I = " << ina2.readCurrent()/1000 << "\n";
  }
  std::cout << std::endl;
  bcm2835_i2c_end();
  bcm2835_close();
  return 0;
}