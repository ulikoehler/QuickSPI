#include "QuickI2C.h"

const char* QuickI2CStatusToString(QuickI2CStatus status) {
    if(status == QuickI2CStatus::OK) {
        return "OK";
    } else if(status == QuickI2CStatus::SlaveNotResponding) {
        return "Slave not Responding";
    } else if(status == QuickI2CStatus::DataTimeout) {
        return "Data timeout";
    } else if(status == QuickI2CStatus::VerifyMismatch) {
        return "Verify mismatch";
    } else {
        return "Unknown";
    }
}

QuickI2CDevice::QuickI2CDevice(uint16_t address, TwoWire& wire, uint32_t i2cClockSpeed) : wire(wire), address(address), i2cClockSpeed(i2cClockSpeed) {
}

QuickI2CStatus QuickI2CDevice::writeData(uint8_t registerAddress, const uint8_t* buf, size_t len) {
    // Configure clock speed & timeout
    wire.setClock(this->i2cClockSpeed);
    // Don't need timeout for write since we will never read
    // Transmit address
    wire.beginTransmission(this->address);
    wire.write(registerAddress);
    wire.write(buf, len);
    wire.endTransmission();
    // Check for missing
    if(wire.lastError() == I2C_ERROR_ACK) {
        return QuickI2CStatus::SlaveNotResponding;
    }
    return QuickI2CStatus::OK;
}

QuickI2CStatus QuickI2CDevice::readData(uint8_t registerAddress, uint8_t* buf, size_t len) {
    // Configure clock speed & timeout
    wire.setClock(this->i2cClockSpeed);
    wire.setTimeout(this->computeTimeout(len));
    // Transmit address
    wire.beginTransmission(this->address);
    wire.write(registerAddress);
    wire.endTransmission();
    // Check for missing
    if(wire.lastError() == I2C_ERROR_ACK) {
        return QuickI2CStatus::SlaveNotResponding;
    }
    // Receive data
    wire.requestFrom(this->address, len);
    size_t actuallyRead = wire.readBytes(buf, len);
    if(actuallyRead < len) {
        // Did not read enough bytes
        this->dataBytesReadUntilTimeout = actuallyRead;
        return QuickI2CStatus::DataTimeout;
    }
    return QuickI2CStatus::OK;
}

QuickI2CStatus QuickI2CDevice::writeAndVerifyData(uint8_t registerAddress, const uint8_t* buf, size_t len, uint8_t* rxbuf) {
    // Try to write - if it fails, do not try to verify
    QuickI2CStatus rc = writeData(registerAddress, buf, len);
    if(rc != QuickI2CStatus::OK) {
        return rc;
    }
    // Insert grace time between write and read
    delay(delayBetweenWriteAndRead);
    // Read back data for verify
    rc = readData(registerAddress, rxbuf, len);
    if(rc != QuickI2CStatus::OK) {
        return rc;
    }
    // Compare data
    if(memcmp(rxbuf, buf, len) != 0) {
        // rxbuf is not the same as buf
        return QuickI2CStatus::VerifyMismatch;
    }
    return QuickI2CStatus::OK;
}

QuickI2CStatus QuickI2CDevice::read8BitRegister(uint8_t registerAddress, uint8_t* buf) {
    return readData(registerAddress, buf, 1);
}

QuickI2CStatus QuickI2CDevice::read16BitRegister(uint8_t registerAddress, uint16_t* buf) {
    return readData(registerAddress, (uint8_t*)buf, 2);
}

QuickI2CStatus QuickI2CDevice::read32BitRegister(uint8_t registerAddress, uint32_t* buf) {
    return readData(registerAddress, (uint8_t*)buf, 4);
}

QuickI2CStatus QuickI2CDevice::write8BitRegister(uint8_t registerAddress, uint8_t value) {
    return writeData(registerAddress, &value, 1);
}

QuickI2CStatus QuickI2CDevice::write16BitRegister(uint8_t registerAddress, uint16_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 2);
}

QuickI2CStatus QuickI2CDevice::write32BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeData(registerAddress, (uint8_t*)&value, 4);
}

QuickI2CStatus QuickI2CDevice::writeAndVerify8BitRegister(uint8_t registerAddress, uint8_t value, uint8_t* rxbuf) {
    return writeAndVerifyData(registerAddress, &value, 1, rxbuf);
}

QuickI2CStatus QuickI2CDevice::writeAndVerify16BitRegister(uint8_t registerAddress, uint16_t value, uint8_t* rxbuf) {
    return writeAndVerifyData(registerAddress, (uint8_t*)&value, 2, rxbuf);
}

QuickI2CStatus QuickI2CDevice::writeAndVerify32BitRegister(uint8_t registerAddress, uint32_t value, uint8_t* rxbuf) {
    return writeAndVerifyData(registerAddress, (uint8_t*)&value, 4, rxbuf);
}


uint32_t QuickI2CDevice::computeTimeout(uint32_t bytesToTransfer) {
    uint32_t numBits = bytesToTransfer * 9; // 8 data bits + 1 ACK/NACK bit per byte
    // NOTE: We avoid to use floating point arithmetic here.
    uint32_t durationMilliseconds = numBits * 1000 / this->i2cClockSpeed; // This will ALWAYS be rounded down!
    // Give an extra millisecond so we have ensured to always round up
    return durationMilliseconds + 2;
}