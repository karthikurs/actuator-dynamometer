#include "I2CDevice.h"
#include <iostream>

//#define DEBUG_SERIAL Serial

/*!
 *    @brief  Create an I2C device at a given address
 *    @param  addr The 7-bit I2C address for the device
 *    @param  theWire The I2C bus to use, defaults to &Wire
 */
I2CDevice::I2CDevice(uint8_t addr) {
  _addr = addr;
  _begun = false;
  _maxBufferSize = 250; // as defined in Wire.h's RingBuffer
  bcm2835_i2c_setSlaveAddress(_addr);
}

/*!
 *    @brief  Initializes and does basic address detection
 *    @param  addr_detect Whether we should attempt to detect the I2C address
 * with a scan. 99% of sensors/devices don't mind but once in a while, they spaz
 * on a scan!
 *    @return True if I2C initialized and a device with the addr found
 */
bool I2CDevice::begin(bool addr_detect) {
  // std::cerr << "trying to begin in I2CDevice" << std::endl;
  // _begun = bcm2835_i2c_begin();
  // bcm2835_i2c_setSlaveAddress(_addr);
  // bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
  bcm2835_i2c_set_baudrate(390000);
  // bcm2835_i2c_setClockDivider(626);

  if (addr_detect) {
    return detected();
  }
  return true;
}

void I2CDevice::set_address(uint8_t addr) {
  bcm2835_i2c_setSlaveAddress(addr);
}


void I2CDevice::set_address() {
  bcm2835_i2c_setSlaveAddress(_addr);
}

/*!
 *    @brief  Scans I2C for the address - note will give a false-positive
 *    if there's no pullups on I2C
 *    @return True if I2C initialized and a device with the addr found
 */
bool I2CDevice::detected(void) {
  // Init I2C if not done yet
  if (!_begun && !begin()) {
    return false;
  }

  std::cerr << "trying to detect" << std::endl;
  // bcm2835_i2c_setSlaveAddress(_addr);

  // A basic scanner, see if it ACK's
  char a;
  return bcm2835_i2c_read(&a, 0) ==  bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK;
  // return true;
}

/*!
 *    @brief  Write a buffer or two to the I2C device. Cannot be more than
 * maxBufferSize() bytes.
 *    @param  buffer Pointer to buffer of data to write. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  len Number of bytes from buffer to write
 *    @param  prefix_buffer Pointer to optional array of data to write before
 * buffer. Cannot be more than maxBufferSize() bytes. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  prefix_len Number of bytes from prefix buffer to write
 *    @param  stop Whether to send an I2C STOP signal on write
 *    @return True if write was successful, otherwise false.
 */
bool I2CDevice::write(uint8_t *buffer, size_t len, bool stop,
                               uint8_t *prefix_buffer,
                               size_t prefix_len) {
  if ((len + prefix_len) > maxBufferSize()) {
    // currently not guaranteed to work if more than 32 bytes!
    // we will need to find out if some platforms have larger
    // I2C buffer sizes :/
    return false;
  }

  // bcm2835_i2c_setSlaveAddress(_addr);

  // Write the prefix data (usually an address)
  if ((prefix_len != 0) && (prefix_buffer != NULL)) {
    if (bcm2835_i2c_write(prefix_buffer, prefix_len) != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
    // if (bcm2835_i2c_write_read_rs(prefix_buffer, prefix_len, prefix_buffer, 0) != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
      return false;
    }
  }

  // Write the data itself
  if (bcm2835_i2c_write(buffer, len) != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
  // if (bcm2835_i2c_write_read_rs(buffer, len, buffer, 0) != bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK) {
    return false;
  }

  // bcm2835_i2c_end();
  return true;
}

/*!
 *    @brief  Read from I2C into a buffer from the I2C device.
 *    Cannot be more than maxBufferSize() bytes.
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  stop Whether to send an I2C STOP signal on read
 *    @return True if read was successful, otherwise false.
 */
bool I2CDevice::read(uint8_t *buffer, size_t len, bool stop) {
  // bcm2835_i2c_setSlaveAddress(_addr);
  return bcm2835_i2c_read(buffer, len) == bcm2835I2CReasonCodes::BCM2835_I2C_REASON_OK;
}

/*!
 *    @brief  Write some data, then read some data from I2C into another buffer.
 *    Cannot be more than maxBufferSize() bytes. The buffers can point to
 *    same/overlapping locations.
 *    @param  write_buffer Pointer to buffer of data to write from
 *    @param  write_len Number of bytes from buffer to write.
 *    @param  read_buffer Pointer to buffer of data to read into.
 *    @param  read_len Number of bytes from buffer to read.
 *    @param  stop Whether to send an I2C STOP signal between the write and read
 *    @return True if write & read was successful, otherwise false.
 */
bool I2CDevice::write_then_read(const uint8_t *write_buffer,
                                         size_t write_len, uint8_t *read_buffer,
                                         size_t read_len, bool stop) {
  if (!write(write_buffer, write_len, stop)) {
    return false;
  }
  return read(read_buffer, read_len);
}

bool I2CDevice::w_read_rs(const uint8_t *regaddr,
                          uint8_t *read_buffer,
                          size_t read_len) {
  return bcm2835_i2c_read_register_rs(regaddr, read_buffer, read_len);
}

bool I2CDevice::read_reg(const uint8_t *regaddr,
                          uint8_t *read_buffer,
                          size_t read_len) {
  // bcm2835_i2c_write_read_rs(regaddr, 1, read_buffer, 0);
  bcm2835_i2c_write(regaddr, 1);
  bool success = bcm2835_i2c_read(read_buffer, read_len);
  return success;
}


/*!
 *    @brief  Returns the 7-bit address of this device
 *    @return The 7-bit address of this device
 */
uint8_t I2CDevice::address(void) { return _addr; }

/*!
 *    @brief  Change the I2C clock speed to desired (relies on
 *    underlying Wire support!
 *    @param desiredclk The desired I2C SCL frequency
 *    @return True if this platform supports changing I2C speed.
 *    Not necessarily that the speed was achieved!
 */
bool I2CDevice::setSpeed(uint32_t desiredclk) {
  bcm2835_i2c_set_baudrate(desiredclk);
  return true;
}

uint16_t I2CDevice::read_bits(const uint8_t *regaddr,
                          uint8_t bits,
                          uint8_t shift) {
  read_reg(regaddr, _data_buf, 2);
  uint16_t val = ((_data_buf[0] << 8) | _data_buf[1]);

  val >>= shift;
  return val & ((1 << (bits)) - 1);
}

void I2CDevice::write_bits(const uint8_t *regaddr,
                          uint8_t bits,
                          uint8_t shift,
                          uint16_t data) {
  read_reg(regaddr, _data_buf, 2);
  uint16_t val = ((_data_buf[0] << 8) | _data_buf[1]);

  // mask off the data before writing
  uint16_t mask = (1 << (bits)) - 1;
  data &= mask;

  mask <<= shift;
  val &= ~mask;          // remove the current data at that spot
  val |= data << shift; // and add in the new data

  _data_buf[0] = (val & 0xFF00) >> 8;
  _data_buf[1] = val & 0xFF;

  write(_data_buf, 2);
}