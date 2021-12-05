#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <SPI.h>

/**
 * Define a Read-Write register and its associated functions on
 * 
 * DO NOT terminate these macro calls with a semicolon!
 */
#define QUICKSPI_DEFINE_REGISTER8_RW(name, addr)\
static constexpr uint8_t name = addr;\
inline uint8_t read##name(uint8_t* out) {return read8BitRegister((addr));}\
inline void write##name(uint8_t val) {return write8BitRegister((addr), val);}\
inline bool writeAndVerify##name(uint8_t val) {return writeAndVerify8BitRegister((addr), val);}
#define QUICKSPI_DEFINE_REGISTER16_RW(name, addr)\
static constexpr uint8_t name = addr;\
inline uint16_t read##name() {return read16BitRegister((addr));}\
inline void write##name(uint16_t val) {return write16BitRegister((addr), val);}\
inline bool writeAndVerify##name(uint16_t val) {return writeAndVerify16BitRegister((addr), val);}
#define QUICKSPI_DEFINE_REGISTER24_RW(name, addr)\
static constexpr uint8_t name = addr;\
inline uint32_t read##name() {return read24BitRegister((addr));}\
inline void write##name(uint32_t val) {return write24BitRegister((addr), val);}\
inline bool writeAndVerify##name(uint32_t val) {return writeAndVerify24BitRegister((addr), val);}
#define QUICKSPI_DEFINE_REGISTER32_RW(name, addr)\
static constexpr uint8_t name = addr;\
inline void read##name(uint32_t* out) {return read32BitRegister((addr));}\
inline void write##name(uint32_t val) {return write32BitRegister((addr), val);}\
inline bool writeAndVerify##name(uint32_t val) {return writeAndVerify32BitRegister((addr), val);}

// Read-only register definitions
#define QUICKSPI_DEFINE_REGISTER8_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline void read##name(uint8_t* out) {return read8BitRegister((addr));}
#define QUICKSPI_DEFINE_REGISTER16_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline void read##name(uint16_t* out) {return read16BitRegister((addr));}
#define QUICKSPI_DEFINE_REGISTER24_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline void read##name(uint32_t* out) {return read24BitRegister((addr));}
#define QUICKSPI_DEFINE_REGISTER32_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline void read##name(uint32_t* out) {return read32BitRegister((addr));}

/**
 * Represents a single device on the SPI bus.
 * 
 * You can have multiple QuickSPIDevice instances, each with a different clock speed.
 * QuickSPIDevice will managed the clock speed, but it will not automagically manage the signal integrity!
 * You need to ensure that the slave select signals are never asserted accidentally and
 * when talking to a specific device, that the SCLK, MISO & MOSI signals are not distorted and
 * that the 
 * 
 * Note: This class is not inherently thread-safe and does not perform locking by itself,
 * it inherits its thread safety from the SPI library.
 * Either ensure that not concurrent accesses are possible or perform locking.
 */
class QuickSPIDevice {
public:
    QuickSPIDevice(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings);

    uint8_t read8BitRegister(uint8_t registerAddress);
    uint16_t read16BitRegister(uint8_t registerAddress);
    uint32_t read32BitRegister(uint8_t registerAddress);
    void readData(uint8_t registerAddress, uint8_t* buf, size_t len);

    void write8BitRegister(uint8_t registerAddress, uint8_t value);
    void write16BitRegister(uint8_t registerAddress, uint16_t value);
    void write24BitRegister(uint8_t registerAddress, uint32_t value);
    void write32BitRegister(uint8_t registerAddress, uint32_t value);

    /**
     * Write data, storing the received data in the given buffer.
     * NOTE: This will discard data received while transmitting the address byte,
     * storing overall [len] bytes in buf
     */
    void writeAndReceiveData(uint8_t registerAddress, uint8_t* buf, size_t len);
    /**
     * Write data, discarding the received data
     */
    void writeData(uint8_t registerAddress, const uint8_t* buf, size_t len);

    /**
     * @return true if the value which has been read back matches the value written into the device
     * @return false if the value written mismatches the value read back from the register
     */
    bool writeAndVerify8BitRegister(uint8_t registerAddress, uint8_t value);
    bool writeAndVerify16BitRegister(uint8_t registerAddress, uint16_t value);
    bool writeAndVerify24BitRegister(uint8_t registerAddress, uint32_t value);
    bool writeAndVerify32BitRegister(uint8_t registerAddress, uint32_t value);
    bool writeAndVerifyData(uint8_t registerAddress, const uint8_t* buf, size_t len);

    /**
     * Milliseconds delay() between write and read during a writeAndVerifyData()
     * 
     */
    uint32_t delayBetweenWriteAndRead = 1;

protected:
    SPIClass& spi;
    uint8_t ssPin;
    SPISettings settings;
};