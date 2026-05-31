#pragma once


// Automatic driver selection - use Arduino only if Arduino is detected, else use ESP-IDF
#if !defined(QUICKSPI_DRIVER_ARDUINO) && !defined(QUICKSPI_DRIVER_ESPIDF)
#if defined(ARDUINO) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_SAMD)
#define QUICKSPI_DRIVER_ARDUINO
#else
#define QUICKSPI_DRIVER_ESPIDF
#endif
#endif

#ifdef QUICKSPI_DRIVER_ARDUINO
#include <Arduino.h>
#include <SPI.h>
#elif defined(QUICKSPI_DRIVER_ESPIDF)
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

#include <stdint.h>
#include <stddef.h>
#include <type_traits>
#include <string.h> // memcpy
#include <algorithm> // std::max

// Static scratch buffer size (override before including this header if desired)
#ifndef QUICKSPI_STATIC_BUFFER_SIZE
    #ifdef CONFIG_QUICKSPI_STATIC_BUFFER_SIZE
        #define QUICKSPI_STATIC_BUFFER_SIZE CONFIG_QUICKSPI_STATIC_BUFFER_SIZE
    #else
        #define QUICKSPI_STATIC_BUFFER_SIZE 16
    #endif
#endif

#ifndef QUICKSPI_ENABLE_DYNAMIC_FALLBACK
    #ifdef CONFIG_QUICKSPI_ENABLE_DYNAMIC_FALLBACK
        #define QUICKSPI_ENABLE_DYNAMIC_FALLBACK 1
    #endif
#endif

#ifndef QUICKSPI_ENABLE_MEMORY_CANARY
    #ifdef CONFIG_QUICKSPI_ENABLE_MEMORY_CANARY
        #define QUICKSPI_ENABLE_MEMORY_CANARY 1
    #endif
#endif

#if !defined(QUICKSPI_NO_ENUM_OPERATIONS) && !defined(QUICKI2C_ENUM_OPERATIONS)
#define QUICKSPI_ENUM_OPERATIONS

/**
 * @brief Define ORing of two enum classes (register definitions)
 */
template<class T>
constexpr typename std::enable_if<std::is_enum<T>::value, T>::type operator|(T lhs, T rhs) 
{
    return static_cast<T>(
        static_cast<typename std::underlying_type<T>::type>(lhs) | 
        static_cast<typename std::underlying_type<T>::type>(rhs));
}

/**
 * @brief Define ORing of an enum class and an integer
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator|(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) | rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator|(I lhs, T rhs)
{
    return lhs | static_cast<typename std::underlying_type<T>::type>(rhs);
}

/**
 * @brief Define ORing of an enum class and an integer
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator&(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) & rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator&(I lhs, T rhs)
{
    return lhs & static_cast<typename std::underlying_type<T>::type>(rhs);
}

/**
 * @brief Define adding of enum classes (register definitions) and numbers
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator+(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) + rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator+(I lhs, T rhs) 
{
    return lhs + static_cast<typename std::underlying_type<T>::type>(rhs);
}

/**
 * @brief Define subtraction of enum classes (register definitions) and numbers
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator-(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) - rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator-(I lhs, T rhs) 
{
    return lhs - static_cast<typename std::underlying_type<T>::type>(rhs);

}

/**
 * @brief Define multiplication of enum classes (register definitions) and numbers
 */
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator*(T lhs, I rhs) 
{
    return static_cast<typename std::underlying_type<T>::type>(lhs) * rhs;
}
// Reversed argument order
template<class T, class I>
constexpr typename std::enable_if<std::is_enum<T>::value && std::is_integral<I>::value, I>::type operator*(I lhs, T rhs) 
{
    return lhs * static_cast<typename std::underlying_type<T>::type>(rhs);
}

#endif

#if !defined(QUICKI2C_UTILS)
#define QUICKSPI_UTILS

// Pre/postprocess functions that do nothing
inline uint8_t noop(uint8_t address, uint8_t rawValue) {return rawValue;}
inline uint16_t noop(uint8_t address, uint16_t rawValue) {return rawValue;}
inline uint32_t noop(uint8_t address, uint32_t rawValue) {return rawValue;}

// Pre/postprocess functions that swap the byte order (but do not change the bit order)
inline uint16_t invertByteorder16(uint8_t address, uint16_t rawValue) {return __builtin_bswap16(rawValue);}
inline uint32_t invertByteorder24(uint8_t address, uint32_t rawValue) {return __builtin_bswap32(rawValue) >> 8;}
inline uint32_t invertByteorder32(uint8_t address, uint32_t rawValue) {return __builtin_bswap32(rawValue);}

#endif

typedef uint8_t (*Postprocessor8Bit)(uint8_t, uint8_t);
typedef uint16_t (*Postprocessor16Bit)(uint8_t, uint16_t);
typedef uint32_t (*Postprocessor24Bit)(uint8_t, uint32_t);
typedef uint32_t (*Postprocessor32Bit)(uint8_t, uint32_t);

#define _DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
static constexpr uint8_t name##ReadAddress = (raddr);\
static constexpr uint8_t name##WriteAddress = (waddr);\

/**
 * Define a Read-Write register and its associated functions on
 * 
 * DO NOT terminate these macro calls with a semicolon!
 */
#define QUICKSPI_DEFINE_REGISTER8_RW(name, raddr, waddr)\
enum class name : uint8_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline uint8_t read##name() {return postprocessRead8((raddr), read8BitRegister((raddr)));}\
inline void write##name(uint8_t val) {write8BitRegister((waddr), postprocessWrite8((waddr), val));}\
inline void write##name(name val) {write8BitRegister((waddr), postprocessWrite8((waddr), static_cast<uint8_t>(val)));}\
inline bool writeAndVerify##name(uint8_t val) {return writeAndVerify8BitRegister((raddr), (waddr), postprocessWrite8((waddr), val));}\
enum class name : uint8_t

#define QUICKSPI_DEFINE_REGISTER16_RW(name, raddr, waddr)\
enum class name : uint16_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline uint16_t read##name() {return postprocessRead16((raddr), read16BitRegister((raddr)));}\
inline void write##name(uint16_t val) {write16BitRegister((waddr), postprocessWrite16((waddr), val));}\
inline void write##name(name val) {write16BitRegister((waddr), postprocessWrite16((waddr), static_cast<uint16_t>(val)));}\
inline bool writeAndVerify##name(uint16_t val) {return writeAndVerify16BitRegister((raddr), (waddr), postprocessWrite16((waddr), val));}\
enum class name : uint16_t

#define QUICKSPI_DEFINE_REGISTER24_RW(name, raddr, waddr)\
enum class name : uint32_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline uint32_t read##name() {return postprocessRead24((raddr), read24BitRegister((raddr)));}\
inline void write##name(uint32_t val) {return write24BitRegister((waddr), postprocessWrite24((waddr), val));}\
inline void write##name(name val) {return write24BitRegister((waddr), postprocessWrite24((waddr), static_cast<uint32_t>(val)));}\
inline bool writeAndVerify##name(uint32_t val) {return writeAndVerify24BitRegister((raddr), (waddr), postprocessWrite24((waddr), val));}\
enum class name : uint32_t

#define QUICKSPI_DEFINE_REGISTER32_RW(name, raddr, waddr)\
enum class name : uint32_t;\
_DEFINE_READ_WRITE_ADDRESS_MEMBERS(name, raddr, waddr)\
inline uint32_t read##name() {return postprocessRead32((raddr), read32BitRegister((raddr)));}\
inline void write##name(uint32_t val) {return write32BitRegister((waddr), postprocessWrite32((waddr), val));}\
inline void write##name(name val) {return write32BitRegister((waddr), postprocessWrite32((waddr), static_cast<uint32_t>(val)));}\
inline bool writeAndVerify##name(uint32_t val) {return writeAndVerify32BitRegister((raddr), (waddr), postprocessWrite32((waddr), val));}\
enum class name : uint32_t

// Read-only register definitions
#define QUICKSPI_DEFINE_REGISTER8_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline uint8_t read##name() {return postprocessRead8((addr), read8BitRegister((addr)));}\
enum class name : uint8_t

#define QUICKSPI_DEFINE_REGISTER16_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline uint16_t read##name() {return postprocessRead16((addr), read16BitRegister((addr)));}\
enum class name : uint16_t

#define QUICKSPI_DEFINE_REGISTER24_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline uint32_t read##name() {return postprocessRead24((addr), read24BitRegister((addr)));}\
enum class name : uint32_t

#define QUICKSPI_DEFINE_REGISTER32_RO(name, addr)\
static constexpr uint8_t name##Address = addr;\
inline uint32_t read##name() {return postprocessRead32((addr), read32BitRegister((addr)));}\
enum class name : uint32_t

/**
 * Represents a single device on the SPI bus.
 *
 * You can have multiple QuickSPIDevice instances, each with a different clock speed.
 * QuickSPIDevice will managed the clock speed, but it will not automagically manage the signal integrity!
 * You need to ensure that the slave select signals are never asserted accidentally and
 * when talking to a specific device, that the SCLK, MISO & MOSI signals are not distorted and
 * that the delay from SCLK output to MISO input is not too long.
 *
 * This class uses a per-instance static scratch buffer for small transactions.
 * Transactions that fit within BufferSize (default QUICKSPI_STATIC_BUFFER_SIZE = 16 bytes) avoid
 * dynamic allocation entirely. Larger transactions fall back to new[] allocation
 * if QUICKSPI_ENABLE_DYNAMIC_FALLBACK is enabled (default).
 *
 * Note: This class is not inherently thread-safe and does not perform locking by itself,
 * it inherits its thread safety from the SPI library.
 * Either ensure that not concurrent accesses are possible or perform locking.
 */
template<size_t BufferSize = QUICKSPI_STATIC_BUFFER_SIZE>
class QuickSPIDevice {
public:
#ifdef QUICKSPI_DRIVER_ARDUINO
    QuickSPIDevice(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings);
#elif defined(QUICKSPI_DRIVER_ESPIDF)
    QuickSPIDevice(spi_host_device_t host, gpio_num_t cs_pin, uint32_t clock_speed_hz, uint8_t mode = 0);
    QuickSPIDevice(spi_host_device_t host, spi_device_interface_config_t* device);
#endif

    // Define a member in your class:
    // constexpr static bool invertReadByteOrder = false;
    // constexpr static bool invertWriteByteOrder = false;

    virtual uint8_t read8BitRegister(uint8_t registerAddress);
    virtual uint16_t read16BitRegister(uint8_t registerAddress);
    virtual uint32_t read24BitRegister(uint8_t registerAddress);
    virtual uint32_t read32BitRegister(uint8_t registerAddress);
    virtual void readRegister(uint8_t registerAddress, uint8_t* buf, size_t len);

    virtual void write8BitRegister(uint8_t registerAddress, uint8_t value);
    virtual void write16BitRegister(uint8_t registerAddress, uint16_t value);
    virtual void write24BitRegister(uint8_t registerAddress, uint32_t value);
    virtual void write32BitRegister(uint8_t registerAddress, uint32_t value);

    /**
     * Write data, storing the received data in the given buffer.
     * NOTE: This will discard data received while transmitting the address byte,
     * storing overall [len] bytes in buf
     */
    virtual void writeAndReadRegister(uint8_t registerAddress, uint8_t* buf, size_t len);
    /**
     * Write data, discarding the received data
     */
    virtual void writeRegister(uint8_t registerAddress, const uint8_t* buf, size_t len);

    /**
     * Write raw data directly to SPI without register address
     */
    virtual void writeRawData(const uint8_t* txbuf, size_t len);

    /**
     * Write and read raw data directly to/from SPI without register address
     * @param trxbuf Buffer containing data to transmit, will be overwritten with received data
     * @param txlen Number of bytes to transmit
     * @param rxlen Number of bytes to receive (buffer must be at least max(txlen, rxlen) bytes)
     */
    virtual void writeReadRawData(uint8_t* trxbuf, size_t txlen, size_t rxlen);

    /**
     * @brief Writes data to a register and verifies if the data has been written correctly by reading back the register
     * and comparing with the original value.
     *
     * @return true if the value which has been read back matches the value written into the device
     * @return false if the value written mismatches the value read back from the register
     */
    virtual bool writeAndVerify8BitRegister(uint8_t readAddress, uint8_t writeAddress, uint8_t value);
    virtual bool writeAndVerify16BitRegister(uint8_t readAddress, uint8_t writeAddress, uint16_t value);
    virtual bool writeAndVerify24BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value);
    virtual bool writeAndVerify32BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value);
    virtual bool writeAndVerifyData(uint8_t readAddress,uint8_t writeAddress, const uint8_t* buf, size_t len);

    /**
     * Milliseconds delay() between write and read during a writeAndVerifyData()
     *
     */
    uint32_t delayBetweenWriteAndRead = 1;

protected:
    static constexpr uint16_t CanaryValue = 0xCAFE;

    /**
     * @brief Acquire a buffer pointer for a transaction of the given size.
     * @param size Required buffer size in bytes.
     * @param buffer Output pointer to the buffer (scratch or heap-allocated).
     * @return true if buffer was successfully acquired, false on failure.
     */
    bool acquireBuffer(size_t size, uint8_t*& buffer);

    /**
     * @brief Release a buffer previously acquired via acquireBuffer().
     * @param buffer Pointer to the buffer.
     * @param size Size that was passed to acquireBuffer().
     */
    void releaseBuffer(uint8_t* buffer, size_t size);

#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
    /**
     * @brief Write the canary value to the end of the scratch buffer.
     */
    void writeCanary();

    /**
     * @brief Check the canary value at the end of the scratch buffer.
     * Emits a prominent error log if the canary has been overwritten.
     */
    void checkCanary();
#endif

    /**
     * @brief Static scratch buffer used to avoid dynamic allocation for small transactions.
     * When the canary is enabled, two extra bytes are reserved for the canary value.
     */
#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
    uint8_t _scratchBuffer[BufferSize + sizeof(uint16_t)];
#else
    uint8_t _scratchBuffer[BufferSize];
#endif
#ifdef QUICKSPI_DRIVER_ARDUINO
    SPIClass& spi;
    uint8_t ssPin;
    SPISettings spiSettings;
#elif defined(QUICKSPI_DRIVER_ESPIDF)
    spi_device_handle_t spi_device;
#endif
};

#ifdef QUICKSPI_DRIVER_ARDUINO
template<size_t BufferSize>
QuickSPIDevice<BufferSize>::QuickSPIDevice(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings): spi(spi), ssPin(ssPin), spiSettings(spiSettings) {}
#elif defined(QUICKSPI_DRIVER_ESPIDF)
template<size_t BufferSize>
QuickSPIDevice<BufferSize>::QuickSPIDevice(spi_host_device_t host, gpio_num_t cs_pin, uint32_t clock_speed_hz, uint8_t mode) {
    spi_device_interface_config_t dev_config = {};
    dev_config.clock_speed_hz = static_cast<int>(clock_speed_hz);
    dev_config.mode = mode;
    dev_config.spics_io_num = cs_pin;
    dev_config.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(host, &dev_config, &spi_device));
}

template<size_t BufferSize>
QuickSPIDevice<BufferSize>::QuickSPIDevice(spi_host_device_t host, spi_device_interface_config_t* device) {
    ESP_ERROR_CHECK(spi_bus_add_device(host, device, &spi_device));
}
#endif

#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::writeCanary() {
    uint16_t canary = CanaryValue;
    memcpy(_scratchBuffer + BufferSize, &canary, sizeof(canary));
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::checkCanary() {
    uint16_t canary;
    memcpy(&canary, _scratchBuffer + BufferSize, sizeof(canary));
    if (canary != CanaryValue) {
#ifdef QUICKSPI_DRIVER_ESPIDF
        ESP_LOGE("QuickSPI", "MEMORY CANARY OVERWRITTEN! Expected 0x%04X, got 0x%04X. Scratch buffer overflow detected!", CanaryValue, canary);
#elif defined(QUICKSPI_DRIVER_ARDUINO)
        Serial.printf("QuickSPI: MEMORY CANARY OVERWRITTEN! Expected 0x%04X, got 0x%04X. Scratch buffer overflow detected!\n", CanaryValue, canary);
#endif
        // Restore canary so the next check can detect any further overflows
        writeCanary();
    }
}
#endif

template<size_t BufferSize>
bool QuickSPIDevice<BufferSize>::acquireBuffer(size_t size, uint8_t*& buffer) {
    if (size <= BufferSize) {
        buffer = _scratchBuffer;
#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
        writeCanary();
#endif
        return true;
    }
#ifdef QUICKSPI_ENABLE_DYNAMIC_FALLBACK
    buffer = new uint8_t[size];
    if (buffer == nullptr) {
#ifdef QUICKSPI_DRIVER_ESPIDF
        ESP_LOGE("QuickSPI", "Failed to allocate %d byte fallback buffer", size);
#elif defined(QUICKSPI_DRIVER_ARDUINO)
        Serial.printf("QuickSPI: Failed to allocate %d byte fallback buffer\n", size);
#endif
        return false;
    }
    return true;
#else
#ifdef QUICKSPI_DRIVER_ESPIDF
    ESP_LOGE("QuickSPI", "Transaction size %d exceeds static buffer (%d) and dynamic fallback is disabled", size, BufferSize);
#elif defined(QUICKSPI_DRIVER_ARDUINO)
    Serial.printf("QuickSPI: Transaction size %d exceeds static buffer (%d) and dynamic fallback is disabled\n", size, BufferSize);
#endif
    buffer = nullptr;
    return false;
#endif
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::releaseBuffer(uint8_t* buffer, size_t size) {
    if (buffer != _scratchBuffer && buffer != nullptr) {
        delete[] buffer;
    }
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::writeRegister(uint8_t registerAddress, const uint8_t* buf, size_t len) {
    size_t totalLen = len + 1;
    uint8_t* txbuf = nullptr;
    if (!acquireBuffer(totalLen, txbuf)) {
        return;
    }
    txbuf[0] = registerAddress;
    memcpy(txbuf + 1, buf, len);

    #ifdef QUICKSPI_DEBUG_WRITES
    Serial.printf("QuickSPI write of size 1+%d of register %02x\r\n", len, registerAddress);
    #endif

    // Use raw data method
    writeRawData(txbuf, totalLen);

#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
    checkCanary();
#endif

    // Cleanup
    releaseBuffer(txbuf, totalLen);
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::writeRawData(const uint8_t* txbuf, size_t len) {

    #ifdef QUICKSPI_DEBUG_WRITES
    Serial.printf("QuickSPI raw write of size %d\r\n", len);
    for (size_t i = 0; i < len; i++)
    {
        Serial.printf(" -- TX byte %d: %02x\r\n", i, txbuf[i]);
    }
    #endif

    // SPI transaction
#ifdef QUICKSPI_DRIVER_ARDUINO
    uint8_t* trxbuf = nullptr;
    if (!acquireBuffer(len, trxbuf)) {
        return;
    }
    // Copy source data
    memcpy(trxbuf, txbuf, len);

    spi.beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi.transfer(trxbuf, len);
    digitalWrite(ssPin, HIGH);
    spi.endTransaction();

#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
    checkCanary();
#endif

    // Cleanup
    releaseBuffer(trxbuf, len);
#elif defined(QUICKSPI_DRIVER_ESPIDF)
    // Transfer directly using txbuf
    spi_transaction_t trans = {};
    trans.length = len * 8; // length in bits
    trans.tx_buffer = txbuf;
    ESP_ERROR_CHECK(spi_device_transmit(spi_device, &trans));
#endif

}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::writeReadRawData(uint8_t* trxbuf, size_t txlen, size_t rxlen) {

    #if defined(QUICKSPI_DEBUG_WRITES) || defined(QUICKSPI_DEBUG_READS)
    Serial.printf("QuickSPI raw write/read of tx size %d, rx size %d\r\n", txlen, rxlen);
    for (size_t i = 0; i < txlen; i++)
    {
        Serial.printf(" -- TX byte %d: %02x\r\n", i, trxbuf[i]);
    }
    #endif

    // SPI transaction
#ifdef QUICKSPI_DRIVER_ARDUINO
    size_t maxlen = (txlen > rxlen) ? txlen : rxlen;
    spi.beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi.transfer(trxbuf, maxlen);
    digitalWrite(ssPin, HIGH);
    spi.endTransaction();
#elif defined(QUICKSPI_DRIVER_ESPIDF)
    spi_transaction_t trans = {};
    trans.length = std::max(txlen, rxlen) * 8; // length in bits
    trans.tx_buffer = trxbuf;
    trans.rx_buffer = trxbuf;
    ESP_ERROR_CHECK(spi_device_transmit(spi_device, &trans));
#endif

    #if defined(QUICKSPI_DEBUG_WRITES) || defined(QUICKSPI_DEBUG_READS)
    for (size_t i = 0; i < rxlen; i++)
    {
        Serial.printf(" -- RX byte %d: %02x\r\n", i, trxbuf[i]);
    }
    #endif
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::writeAndReadRegister(uint8_t registerAddress, uint8_t* buf, size_t len) {
    size_t totalLen = len + 1;
    uint8_t* trxbuf = nullptr;
    if (!acquireBuffer(totalLen, trxbuf)) {
        return;
    }
    trxbuf[0] = registerAddress;
    memcpy(trxbuf + 1, buf, len);

    #if defined(QUICKSPI_DEBUG_WRITES) || defined(QUICKSPI_DEBUG_READS)
    Serial.printf("QuickSPI read/write of size 1+%d of register %02x\r\n", len, registerAddress);
    #endif

    // Use raw data method
    writeReadRawData(trxbuf, totalLen, totalLen);

#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
    checkCanary();
#endif

    // Copy received data (skip first byte which is the register address response)
    memcpy(buf, trxbuf + 1, len);

    // Cleanup
    releaseBuffer(trxbuf, totalLen);
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::readRegister(uint8_t registerAddress, uint8_t* buf, size_t len) {
    size_t totalLen = len + 1;
    uint8_t* trxbuf = nullptr;
    if (!acquireBuffer(totalLen, trxbuf)) {
        return;
    }
    trxbuf[0] = registerAddress;

    #ifdef QUICKSPI_DEBUG_READS
    Serial.printf("QuickSPI read of size 1+%d of register %02x\r\n", len, registerAddress);
    #endif

    // Use raw data method (send 1 byte, receive len+1 bytes)
    writeReadRawData(trxbuf, 1, totalLen);

#ifdef QUICKSPI_ENABLE_MEMORY_CANARY
    checkCanary();
#endif

    // Copy received data (skip first byte which is the register address response)
    memcpy(buf, trxbuf + 1, len);

    // Cleanup
    releaseBuffer(trxbuf, totalLen);
}

template<size_t BufferSize>
bool QuickSPIDevice<BufferSize>::writeAndVerifyData(uint8_t readAddress, uint8_t writeAddress, const uint8_t* buf, size_t len) {
    // Try to write - if it fails, do not try to verify
    writeRegister(writeAddress, buf, len);
    // Insert grace time between write and read
#ifdef QUICKSPI_DRIVER_ARDUINO
    delay(delayBetweenWriteAndRead);
#elif defined(QUICKSPI_DRIVER_ESPIDF)
    vTaskDelay(pdMS_TO_TICKS(delayBetweenWriteAndRead));
#endif
    // Read back data for verify
    uint8_t* rxbuf = nullptr;
    if (!acquireBuffer(len, rxbuf)) {
        return false;
    }
    readRegister(readAddress, rxbuf, len);
    // Compare data
    bool result = memcmp(rxbuf, buf, len) == 0; // true => rx data is the same as tx data
    // cleanup
    releaseBuffer(rxbuf, len);
    return result;
}

template<size_t BufferSize>
uint8_t QuickSPIDevice<BufferSize>::read8BitRegister(uint8_t registerAddress) {
    uint8_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 1);
    return ret;
}

template<size_t BufferSize>
uint16_t QuickSPIDevice<BufferSize>::read16BitRegister(uint8_t registerAddress) {
    uint16_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 2);
    return ret;
}

template<size_t BufferSize>
uint32_t QuickSPIDevice<BufferSize>::read24BitRegister(uint8_t registerAddress) {
    uint32_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 3);
    return ret;
}

template<size_t BufferSize>
uint32_t QuickSPIDevice<BufferSize>::read32BitRegister(uint8_t registerAddress) {
    uint32_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 4);
    return ret;
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::write8BitRegister(uint8_t registerAddress, uint8_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 1);
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::write16BitRegister(uint8_t registerAddress, uint16_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 2);
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::write24BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 3);
}

template<size_t BufferSize>
void QuickSPIDevice<BufferSize>::write32BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 4);
}

template<size_t BufferSize>
bool QuickSPIDevice<BufferSize>::writeAndVerify8BitRegister(uint8_t readAddress, uint8_t writeAddress, uint8_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 1);
}

template<size_t BufferSize>
bool QuickSPIDevice<BufferSize>::writeAndVerify16BitRegister(uint8_t readAddress, uint8_t writeAddress, uint16_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 2);
}

template<size_t BufferSize>
bool QuickSPIDevice<BufferSize>::writeAndVerify24BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 3);
}

template<size_t BufferSize>
bool QuickSPIDevice<BufferSize>::writeAndVerify32BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 4);
}