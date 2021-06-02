#include <iostream>
#include "Adafruit_ADS1X15.h"


int main() {
  Adafruit_ADS1015 ads;
  uint16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;
  if (!bcm2835_init()) {
    std::cout << "bcm2835_init failed. Are you running as root??\n" << std::endl;
    return 1;
  }
  ads.begin(0x48);
  ads.setGain(adsGain_t::GAIN_ONE);
  ads.setDataRate(RATE_ADS1015_3300SPS);
  std::cout << "set ads params" << std::endl;
  for(size_t ii = 0; ii < 10; ++ii) {

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    volts0 = ads.computeVolts(adc0);
    volts1 = ads.computeVolts(adc1);
    volts2 = ads.computeVolts(adc2);
    volts3 = ads.computeVolts(adc3);
    std::cout << volts0 << ", " << volts1 << ", " << volts2 << ", " << volts3 << std::endl;
    std::cout << adc0 << ", " << adc1 << ", " << adc2 << ", " << adc3 << std::endl;
  }
  std::cout << std::endl;
  bcm2835_i2c_end();
  bcm2835_close();
  return 0;
}