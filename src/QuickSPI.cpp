#include "QuickSPI.h"
#include <string.h> // memcpy

QuickSPIDevice::QuickSPIDevice(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings): spi(spi), ssPin(ssPin), spiSettings(spiSettings) {}

void QuickSPIDevice::writeData(uint8_t registerAddress, const uint8_t* buf, size_t len) {
    // Transmit & receive buffer.
    // Length + 1 since we need to 
    // NOTE: Received data will
    uint8_t* trxbuf = new uint8_t[len + 1];
    trxbuf[0] = registerAddress;
    // Copy source data
    memcpy(trxbuf + 1, buf, len);
    // SPI transaction
    spi.beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi.transfer(trxbuf, len + 1);
    digitalWrite(ssPin, HIGH);
    spi.endTransaction();
    // Cleanup. Discards received data
    delete[] trxbuf;
}

void QuickSPIDevice::writeAndReceiveData(uint8_t registerAddress, uint8_t* buf, size_t len) {
    // Transmit & receive buffer.
    // Length + 1 since we need to 
    // NOTE: Received data will
    uint8_t* trxbuf = new uint8_t[len + 1];
    trxbuf[0] = registerAddress;
    // Copy source data
    memcpy(trxbuf + 1, buf, len);
    // SPI transaction
    spi.beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi.transfer(trxbuf, len + 1);
    digitalWrite(ssPin, HIGH);
    spi.endTransaction();
    // Copy received data
    memcpy(buf, trxbuf + 1, len);
    // Cleanup.
    delete[] trxbuf;
}

void QuickSPIDevice::readData(uint8_t registerAddress, uint8_t* buf, size_t len) {
    // Transmit & receive buffer.
    // Length + 1 since we need to 
    // NOTE: Received data will
    uint8_t* trxbuf = new uint8_t[len + 1];
    trxbuf[0] = registerAddress;
    // SPI transaction
    spi.beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi.transfer(trxbuf, len + 1);
    digitalWrite(ssPin, HIGH);
    spi.endTransaction();
    // Copy to destination buffer
    memcpy(buf, trxbuf + 1, len);
    // Cleanup
    delete[] trxbuf;
}

bool QuickSPIDevice::writeAndVerifyData(uint8_t readAddress, uint8_t writeAddress, const uint8_t* buf, size_t len) {
    // Try to write - if it fails, do not try to verify
    writeData(writeAddress, buf, len);
    // Insert grace time between write and read
    delay(delayBetweenWriteAndRead);
    // Read back data for verify
    uint8_t* rxbuf = new uint8_t[len];
    readData(readAddress, rxbuf, len);
    // Compare data
    bool result = memcmp(rxbuf, buf, len) == 0; // true => rx data is the same as tx data
    // cleanup
    delete[] rxbuf;
    return result;
}

uint8_t QuickSPIDevice::read8BitRegister(uint8_t registerAddress) {
    uint8_t ret;
    readData(registerAddress, (uint8_t*)&ret, 1);
    return ret;
}

uint16_t QuickSPIDevice::read16BitRegister(uint8_t registerAddress) {
    uint16_t ret;
    readData(registerAddress, (uint8_t*)&ret, 2);
    return ret;
}

uint32_t QuickSPIDevice::read24BitRegister(uint8_t registerAddress) {
    uint32_t ret;
    readData(registerAddress, (uint8_t*)&ret, 3);
    return ret;
}

uint32_t QuickSPIDevice::read32BitRegister(uint8_t registerAddress) {
    uint32_t ret;
    readData(registerAddress, (uint8_t*)&ret, 4);
    return ret;
}

void QuickSPIDevice::write8BitRegister(uint8_t registerAddress, uint8_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 1);
}

void QuickSPIDevice::write16BitRegister(uint8_t registerAddress, uint16_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 2);
}

void QuickSPIDevice::write24BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 3);
}

void QuickSPIDevice::write32BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 4);
}

bool QuickSPIDevice::writeAndVerify8BitRegister(uint8_t readAddress, uint8_t writeAddress, uint8_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 1);
}

bool QuickSPIDevice::writeAndVerify16BitRegister(uint8_t readAddress, uint8_t writeAddress, uint16_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 2);
}

bool QuickSPIDevice::writeAndVerify24BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 3);
}

bool QuickSPIDevice::writeAndVerify32BitRegister(uint8_t readAddress, uint8_t writeAddress, uint32_t value) {
    return writeAndVerifyData(readAddress, writeAddress, (uint8_t*)&value, 4);
}
                               