#include "QuickSPI.h"
#include <string.h> // memcpy

QuickSPIDevice::QuickSPIDevice(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings): spi(spi), ssPin(ssPin), spiSettings(spiSettings) {}

void QuickSPIDevice::writeData(uint8_t registerAddress, const uint8_t* buf, size_t len) {
    // Transmit & receive buffer.
    // Length + 1 since we need to 
    // NOTE: Received data will
    uint8_t* trxbuf = new uint8_t[len + 1];
    txbuf[0] = registerAddress;
    // Copy source data
    memcpy(trxbuf + 1, buf, len);
    // SPI transaction
    spi->beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi->transfer(trxbuf, len + 1);
    digitalWrite(ssPin, HIGH);
    spi->endTransaction();
    // Cleanup. Discards received data
    delete[] trxbuf;
}

void QuickSPIDevice::writeAndReceiveData(uint8_t registerAddress, uint8_t* buf, size_t len) {
    // Transmit & receive buffer.
    // Length + 1 since we need to 
    // NOTE: Received data will
    uint8_t* trxbuf = new uint8_t[len + 1];
    txbuf[0] = registerAddress;
    // Copy source data
    memcpy(trxbuf + 1, buf, len);
    // SPI transaction
    spi->beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi->transfer(trxbuf, len + 1);
    digitalWrite(ssPin, HIGH);
    spi->endTransaction();
    // Copy received data
    memcpy(buf, trxbuf + 1, len);
    // Cleanup.
    delete[] trxbuf;
}

uint8_t void QuickSPIDevice::readData(uint8_t registerAddress, uint8_t* buf, size_t len) {
    // Transmit & receive buffer.
    // Length + 1 since we need to 
    // NOTE: Received data will
    uint8_t* trxbuf = new uint8_t[len + 1];
    txbuf[0] = registerAddress;
    // SPI transaction
    spi->beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi->transfer(trxbuf, len + 1);
    digitalWrite(ssPin, HIGH);
    spi->endTransaction();
    // Copy to destination buffer
    memcpy(buf, trxbuf + 1, len);
    // Cleanup
    delete[] trxbuf;
}

bool QuickSPIDevice::writeAndVerifyData(uint8_t registerAddress, const uint8_t* buf, size_t len) {
    // Try to write - if it fails, do not try to verify
    writeData(registerAddress, buf, len);
    // Insert grace time between write and read
    delay(delayBetweenWriteAndRead);
    // Read back data for verify
    uint8_t* rxbuf = new uint8_t[len];
    readData(registerAddress, rxbuf, len);
    // Compare data
    bool result = memcmp(rxbuf, buf, len) == 0; // true => rx data is the same as tx data
    // cleanup
    delete[] rxbuf;
    return result;
}

void QuickSPIDevice::read8BitRegister(uint8_t registerAddress) {
    return readData(registerAddress, (uint8_t*)buf, 1);
}

void QuickSPIDevice::read16BitRegister(uint8_t registerAddress) {
    return readData(registerAddress, (uint8_t*)buf, 2);
}

void QuickSPIDevice::read24BitRegister(uint8_t registerAddress) {
    return readData(registerAddress, (uint8_t*)buf, 3);
}

void QuickSPIDevice::read32BitRegister(uint8_t registerAddress, uint32_t* buf) {
    return readData(registerAddress, (uint8_t*)buf, 4);
}

void QuickSPIDevice::write8BitRegister(uint8_t registerAddress, uint8_t value) {
    return writeData(registerAddress, &value, 1);
}

void QuickSPIDevice::write16BitRegister(uint8_t registerAddress, uint16_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 2);
}

void QuickSPIDevice::write24BitRegister(uint8_t registerAddress, uint16_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 3);
}

void QuickSPIDevice::write32BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 4);
}

void QuickSPIDevice::writeAndVerify8BitRegister(uint8_t registerAddress, uint8_t value, uint8_t* rxbuf) {
    return writeAndVerifyData(registerAddress, &value, 1, rxbuf);
}

void QuickSPIDevice::writeAndVerify16BitRegister(uint8_t registerAddress, uint16_t value, uint8_t* rxbuf) {
    return writeAndVerifyData(registerAddress, (uint8_t*)&value, 2, rxbuf);
}

void QuickSPIDevice::writeAndVerify24BitRegister(uint8_t registerAddress, uint32_t value, uint8_t* rxbuf) {
    return writeAndVerifyData(registerAddress, (uint8_t*)&value, 3, rxbuf);
}

void QuickSPIDevice::writeAndVerify32BitRegister(uint8_t registerAddress, uint32_t value, uint8_t* rxbuf) {
    return writeAndVerifyData(registerAddress, (uint8_t*)&value, 4, rxbuf);
}


uint32_t QuickSPIDevice::computeTimeout(uint32_t bytesToTransfer) {
    uint32_t numBits = bytesToTransfer * 9; // 8 data bits + 1 ACK/NACK bit per byte
    // NOTE: We avoid to use floating point arithmetic here.
    uint32_t durationMilliseconds = numBits * 1000 / this->SPIClockSpeed; // This will ALWAYS be rounded down!
    // Give an extra millisecond so we have ensured to always round up
    return durationMilliseconds + 2;
}