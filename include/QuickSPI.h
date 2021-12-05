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
#define QUICKSPI_DEFINE_REGISTER8_RW(name, raddr, waddr)\
static constexpr uint8_t name##ReadAddress = (raddr);\
static constexpr uint8_t name##WriteAddress = (waddr);\
inline uint8_t read##name() {return read8BitRegister((raddr));}\
inline void write##name(uint8_t val) {return write8BitRegister((raddr), val);}\
inline bool writeAndVerify##name(uint8_t val) {return writeAndVerify8BitRegister((raddr), (waddr), val);}
#define QUICKSPI_DEFINE_REGISTER16_RW(name, raddr, waddr)\
static constexpr uint8_t name##ReadAddress = (raddr);\
static constexpr uint8_t name##WriteAddress = (waddr);\
inline uint16_t read##name() {return read16BitRegister((raddr));}\
inline void write##name(uint16_t val) {return write16BitRegister((waddr), val);}\
inline bool writeAndVerify##name(uint16_t val) {return writeAndVerify16BitRegister((raddr), (waddr), val);}
#define QUICKSPI_DEFINE_REGISTER24_RW(name, raddr, waddr)\
static constexpr uint8_t name##ReadAddress = (raddr);\
static constexpr uint8_t name##WriteAddress = (waddr);\
inline uint32_t read##name() {return read24BitRegister((raddr));}\
inline void write##name(uint32_t val) {return write24BitRegister((waddr), val);}\
inline bool writeAndVerify##name(uint32_t val) {return writeAndVerify24BitRegister((raddr), (waddr), val);}
#define QUICKSPI_DEFINE_REGISTER32_RW(name, raddr, waddr)\
static constexpr uint8_t name##ReadAddress = (raddr);\
static constexpr uint8_t name##WriteAddress = (waddr);\
inline void read##name() {return read32BitRegister((raddr));}\
inline void write##name(uint32_t val) {return write32BitRegister((waddr), val);}\
inline bool writeAndVerify##name(uint32_t val) {return writeAndVerify32BitRegister((raddr), (waddr), val);}

// Read-only register definitions
#define QUICKSPI_DEFINE_REGISTER8_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline uint8_t read##name() {return read8BitRegister((addr));}
#define QUICKSPI_DEFINE_REGISTER16_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline uint16_t read##name() {return read16BitRegister((addr));}
#define QUICKSPI_DEFINE_REGISTER24_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline uint32_t read##name() {return read24BitRegister((addr));}
#define QUICKSPI_DEFINE_REGISTER32_RO(name, addr)\
static constexpr uint8_t name = addr;\
inline uint32_t read##name() {return read32BitRegister((addr));}

/**
 * Represents a single device on the SPI bus.
 * 
 * You can have multiple QuickSPIDevice instances, each with a different clock speed.
 * QuickSPIDevice will managed the clock speed, but it will not automagically manage the signal integrity!
 * You need to ensure that the slave select signals are never asserted accidentally and
 * when talking to a specific device, that the SCLK, MISO & MOSI signals are not distorted and
 * that the delay from SCLK output to MISO input is not too long.
 * 
 * This class will perform dynamic memory allocation to allocate buffers.
 * All dynamic memory will be freed before the respective QuickSPIDevice function exits.
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
    uint32_t read24BitRegister(uint8_t registerAddress);
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
    bool writeAndVerify8BitRegister(uint8_t readAddress, uint8_t writeAddress, uint8_t value);
    bool writeAndVerify16BitRegister(uint8_t readAddress, uint8_t writeAddress, uint16_t value);
    bool writeAndVerify24BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value);
    bool writeAndVerify32BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value);
    bool writeAndVerifyData(uint8_t readAddress,uint8_t writeAddress, const uint8_t* buf, size_t len);

    /**
     * Milliseconds delay() between write and read during a writeAndVerifyData()
     * 
     */
    uint32_t delayBetweenWriteAndRead = 1;

protected:
    SPIClass& spi;
    uint8_t ssPin;
    SPISettings spiSettings;
};