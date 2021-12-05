#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <Wire.h>

enum class QuickI2CStatus : int8_t {
    OK = 0,
    /**
     * If you see this error code, check the device address - reason:
     * Missing ACK on the bus, possibly caused by write size mismatch
     */
    SlaveNotResponding = 1,
    /**
     * Device responded to address write but did not send enough/any data within timeout.
     * Check dataBytesReadUntilTimeout!
     */
    DataTimeout = 3,
    /**
     * Write succeeded but reading the register did not result in the correct value.
     * Check the rxbuf to see what the chip returned.
     */
    VerifyMismatch = 4 // Write succeeded, but verify failed
};

/**
 * Define a Read-Write register and its associated functions on
 * 
 * DO NOT terminate these macro calls with a semicolon!
 */
#define QI2C_DEFINE_REGISTER8_RW(name, addr)\
static constexpr uint8_t name = addr;\
inline QuickI2CStatus read##name(uint8_t* out) {return read8BitRegister((addr), out);}\
inline QuickI2CStatus write##name(uint8_t val) {return write8BitRegister((addr), val);}\
inline QuickI2CStatus writeAndVerify##name(uint8_t val, uint8_t* rxbuf) {return writeAndVerify8BitRegister((addr), val, rxbuf);}
#define QI2C_DEFINE_REGISTER16_RW(name, addr)\
static constexpr uint8_t name = addr;\
inline QuickI2CStatus read##name(uint16_t* out) {return read16BitRegister((addr), out);}\
inline QuickI2CStatus write##name(uint16_t val) {return write16BitRegister((addr), val);}\
inline QuickI2CStatus writeAndVerify##name(uint16_t val, uint8_t* rxbuf) {return writeAndVerify16BitRegister((addr), val, rxbuf);}
#define QI2C_DEFINE_REGISTER32_RW(name, addr)\
static constexpr uint8_t name = addr;\
inline QuickI2CStatus read##name(uint32_t* out) {return read32BitRegister((addr), out);}\
inline QuickI2CStatus write##name(uint32_t val) {return write32BitRegister((addr), val);}\
inline QuickI2CStatus writeAndVerify##name(uint32_t val, uint8_t* rxbuf) {return writeAndVerify32BitRegister((addr), val, rxbuf);}

// Read-only register definitions
#define QI2C_DEFINE_REGISTER8_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline QuickI2CStatus read##name(uint8_t* out) {return read8BitRegister((addr), out);}
#define QI2C_DEFINE_REGISTER16_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline QuickI2CStatus read##name(uint16_t* out) {return read16BitRegister((addr), out);}
#define QI2C_DEFINE_REGISTER32_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline QuickI2CStatus read##name(uint32_t* out) {return read32BitRegister((addr), out);}


const char* QuickI2CStatusToString(QuickI2CStatus status);

/**
 * Represents a single device on the I2C bus.
 * 
 * This will automatically manage the timeout and clock speed of the I2C interface,
 * automatically computing a sensible timeout value to use depending on the clock speed:
 * It will not wait the default of one second, but use an adaptive timeout depending on the
 * number of bytes to be transferred
 * 
 * You can have multiple QuickI2CDevice instances, each with a different clock speed.
 * QuickI2CDevice will managed the clock speed, but it will not automagically manage the signal integrity!
 * You need to ensure if using a higher speed like fast mode plus that the slower devices will never
 * think they are being selected. The best way to absolutely ensure proper function is to just always
 * use speeds all devices support, or to hard-disconnect devices not supporting the highest speed
 * using an I2C multiplexer.
 * 
 * Note: This class is not inherently thread-safe and does not perform locking by itself.
 * Either ensure that not concurrent accesses are possible or perform locking.
 */
class QuickI2CDevice {
public:
    QuickI2CDevice(uint16_t address, TwoWire& wire = Wire, uint32_t i2cClockSpeed = 400000);

    QuickI2CStatus read8BitRegister(uint8_t registerAddress, uint8_t* buf);
    QuickI2CStatus read16BitRegister(uint8_t registerAddress, uint16_t* buf);
    QuickI2CStatus read32BitRegister(uint8_t registerAddress, uint32_t* buf);
    QuickI2CStatus readData(uint8_t registerAddress, uint8_t* buf, size_t len);

    QuickI2CStatus write8BitRegister(uint8_t registerAddress, uint8_t value);
    QuickI2CStatus write16BitRegister(uint8_t registerAddress, uint16_t value);
    QuickI2CStatus write32BitRegister(uint8_t registerAddress, uint32_t value);
    QuickI2CStatus writeData(uint8_t registerAddress, const uint8_t* buf, size_t len);

    QuickI2CStatus writeAndVerify8BitRegister(uint8_t registerAddress, uint8_t value, uint8_t* rxbuf);
    QuickI2CStatus writeAndVerify16BitRegister(uint8_t registerAddress, uint16_t value, uint8_t* rxbuf);
    QuickI2CStatus writeAndVerify32BitRegister(uint8_t registerAddress, uint32_t value, uint8_t* rxbuf);
    QuickI2CStatus writeAndVerifyData(uint8_t registerAddress, const uint8_t* buf, size_t len, uint8_t* rxbuf);

    /**
     * This is for debugging purposes.
     * For the previous read, it stores how many data bytes were validly read from the device
     * before the error condition occured.
     * In case the previous operation does not return DeviceNACKDuringData or DataTimeout
     */
    uint32_t dataBytesReadUntilTimeout;

    // Delay in milliseconds between write and read during writeAndVerify.
    uint32_t delayBetweenWriteAndRead = 1;
protected:
    /**
     * Compute the maximum timeout in milliseconds
     * that transferring [bytesToTransfer] could take
     * (plus some extra for safety)
     */
    uint32_t computeTimeout(size_t bytesToTransfer);

    TwoWire& wire;
    /**
     * Device address on the I2C bus.
     */
    uint16_t address;

    uint32_t i2cClockSpeed;
};