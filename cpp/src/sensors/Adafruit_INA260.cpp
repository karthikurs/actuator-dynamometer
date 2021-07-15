/*!
 *  @file Adafruit_INA260.cpp
 *
 *  @mainpage Adafruit INA260 I2C Current and Power sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the INA260 I2C Current and Power sensor
 *
 * 	This is a library for the Adafruit INA260 breakout:
 * 	http://www.adafruit.com/products/4226
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */


#include "Adafruit_INA260.h"
#include <iostream>

uint8_t reg_buf[1];
uint8_t data_buf[4];

/*!
 *    @brief  Instantiates a new INA260 class
 */
Adafruit_INA260::Adafruit_INA260(void) {}

/*!
 *    @brief  Sets up the HW
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  theWire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_INA260::begin(uint8_t i2c_address) {
  i2c_dev = new I2CDevice(i2c_address);

  if (!i2c_dev->begin(false)) {
    return false;
  }

  // std::cerr << "ina begun" << std::endl;

  prime_i2c();
  reset();
  // std::cerr << "ina reset done" << std::endl;
  return true;
}
/**************************************************************************/
/*!
    @brief Resets the harware. All registers are set to default values,
    the same as a power-on reset.
*/
/**************************************************************************/
void Adafruit_INA260::reset(void) {
  reg_buf[0] = INA260_REG_CONFIG;
  i2c_dev->write_bits(reg_buf, 1, 15, 1);
}
/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Current register.
    @return The current current measurement in mA
*/
/**************************************************************************/
float Adafruit_INA260::readCurrent(void) {
  reg_buf[0] = INA260_REG_CURRENT;
  i2c_dev->read_reg(reg_buf, data_buf, 2);
  return 1.25*(int16_t)((data_buf[0] << 8) | data_buf[1]);
}
/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Bus Voltage register.
    @return The current bus voltage measurement in mV
*/
/**************************************************************************/
float Adafruit_INA260::readBusVoltage(void) {
  reg_buf[0] = INA260_REG_BUSVOLTAGE;
  i2c_dev->read_reg(reg_buf, data_buf, 2);
  return 1.25*(int16_t)((data_buf[0] << 8) | data_buf[1]);
}
/**************************************************************************/
/*!
    @brief Reads and scales the current value of the Power register.
    @return The current Power calculation in mW
*/
/**************************************************************************/
float Adafruit_INA260::readPower(void) {
  reg_buf[0] = INA260_REG_POWER;
  i2c_dev->read_reg(reg_buf, data_buf, 2);
  return 10*(int16_t)((data_buf[0] << 8) | data_buf[1]);
}
/**************************************************************************/
/*!
    @brief Returns the current measurement mode
    @return The current mode
*/
/**************************************************************************/
INA260_MeasurementMode Adafruit_INA260::getMode(void) {
  reg_buf[0] = INA260_REG_CONFIG;
  return (INA260_MeasurementMode)i2c_dev->read_bits(reg_buf, 3, 0);
}
/**************************************************************************/
/*!
    @brief Sets a new measurement mode
    @param new_mode
           The new mode to be set
*/
/**************************************************************************/
void Adafruit_INA260::setMode(INA260_MeasurementMode new_mode) {
  reg_buf[0] = INA260_REG_CONFIG;
  i2c_dev->write_bits(reg_buf, 3, 0, (uint16_t)new_mode);
}
/**************************************************************************/
/*!
    @brief Reads the current number of averaging samples
    @return The current number of averaging samples
*/
/**************************************************************************/
INA260_AveragingCount Adafruit_INA260::getAveragingCount(void) {
  reg_buf[0] = INA260_REG_CONFIG;
  return (INA260_AveragingCount)i2c_dev->read_bits(reg_buf, 3, 9);
}
/**************************************************************************/
/*!
    @brief Sets the number of averaging samples
    @param count
           The number of samples to be averaged
*/
/**************************************************************************/
void Adafruit_INA260::setAveragingCount(INA260_AveragingCount count) {
  reg_buf[0] = INA260_REG_CONFIG;
  i2c_dev->write_bits(reg_buf, 3, 9, (uint16_t)count);
}
/**************************************************************************/
/*!
    @brief Reads the current current conversion time
    @return The current current conversion time
*/
/**************************************************************************/
INA260_ConversionTime Adafruit_INA260::getCurrentConversionTime(void) {
  reg_buf[0] = INA260_REG_CONFIG;
  return (INA260_ConversionTime)i2c_dev->read_bits(reg_buf, 3, 3);
}
/**************************************************************************/
/*!
    @brief Sets the current conversion time
    @param time
           The new current conversion time
*/
/**************************************************************************/
void Adafruit_INA260::setCurrentConversionTime(INA260_ConversionTime time) {
  reg_buf[0] = INA260_REG_CONFIG;
  i2c_dev->write_bits(reg_buf, 3, 3, (uint16_t)time);
}
/**************************************************************************/
/*!
    @brief Reads the current bus voltage conversion time
    @return The current bus voltage conversion time
*/
/**************************************************************************/
INA260_ConversionTime Adafruit_INA260::getVoltageConversionTime(void) {
  reg_buf[0] = INA260_REG_CONFIG;
  return (INA260_ConversionTime)i2c_dev->read_bits(reg_buf, 3, 6);
}
/**************************************************************************/
/*!
    @brief Sets the bus voltage conversion time
    @param time
           The new bus voltage conversion time
*/
/**************************************************************************/
void Adafruit_INA260::setVoltageConversionTime(INA260_ConversionTime time) {
  reg_buf[0] = INA260_REG_CONFIG;
  i2c_dev->write_bits(reg_buf, 3, 6, (uint16_t)time);
}

/**************************************************************************/
/*!
    @brief Checks if the most recent one shot measurement has completed
    @return true if the conversion has completed
*/
/**************************************************************************/
bool Adafruit_INA260::conversionReady(void) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  return i2c_dev->read_bits(reg_buf, 1, 3);
}
/**************************************************************************/
/*!
    @brief Reads the current parameter that asserts the ALERT pin
    @return The current parameter that asserts the ALERT PIN
*/
/**************************************************************************/
INA260_AlertType Adafruit_INA260::getAlertType(void) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  return (INA260_AlertType)i2c_dev->read_bits(reg_buf, 6, 10);
}
/**************************************************************************/
/*!
    @brief Sets which parameter asserts the ALERT pin
    @param alert
           The parameter which asserts the ALERT pin
*/
/**************************************************************************/
void Adafruit_INA260::setAlertType(INA260_AlertType alert) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  i2c_dev->write_bits(reg_buf, 6, 10, (uint16_t)alert);
}
/**************************************************************************/
/*!
    @brief Reads the current alert limit setting
    @return The current bus alert limit setting
*/
/**************************************************************************/
float Adafruit_INA260::getAlertLimit(void) {
  reg_buf[0] = INA260_REG_ALERT_LIMIT;
  return 1.25*(float)i2c_dev->read_bits(reg_buf, 16, 0);
}
/**************************************************************************/
/*!
    @brief Sets the Alert Limit
    @param limit
           The new limit that triggers the alert
*/
/**************************************************************************/
void Adafruit_INA260::setAlertLimit(float limit) {
  reg_buf[0] = INA260_REG_ALERT_LIMIT;
  i2c_dev->write_bits(reg_buf, 16, 0, (uint16_t)(limit / 1.25));
}
/**************************************************************************/
/*!
    @brief Reads the current alert polarity setting
    @return The current bus alert polarity setting
*/
/**************************************************************************/
INA260_AlertPolarity Adafruit_INA260::getAlertPolarity(void) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  return (INA260_AlertPolarity)i2c_dev->read_bits(reg_buf, 1, 1);
}
/**************************************************************************/
/*!
    @brief Sets Alert Polarity Bit
    @param polarity
           The polarity of the alert pin
*/
/**************************************************************************/
void Adafruit_INA260::setAlertPolarity(INA260_AlertPolarity polarity) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  i2c_dev->write_bits(reg_buf, 1, 1, (uint16_t)polarity);
}
/**************************************************************************/
/*!
    @brief Reads the current alert latch setting
    @return The current bus alert latch setting
*/
/**************************************************************************/
INA260_AlertLatch Adafruit_INA260::getAlertLatch(void) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  return (INA260_AlertLatch)i2c_dev->read_bits(reg_buf, 1, 0);
}
/**************************************************************************/
/*!
    @brief Sets Alert Latch Bit
    @param state
           The parameter which asserts the ALERT pin
*/
/**************************************************************************/
void Adafruit_INA260::setAlertLatch(INA260_AlertLatch state) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  i2c_dev->write_bits(reg_buf, 1, 0, (uint16_t)state);
}
/**************************************************************************/
/*!
    @brief Checks if the Alert Flag is set
    @return true if the flag is set
*/
/**************************************************************************/
bool Adafruit_INA260::alertFunctionFlag(void) {
  reg_buf[0] = INA260_REG_MASK_ENABLE;
  return i2c_dev->read_bits(reg_buf, 1, 4);
}

void Adafruit_INA260::prime_i2c() {
  i2c_dev->set_address();
}