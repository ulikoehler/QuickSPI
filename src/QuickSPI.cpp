#include "QuickSPI.h"
#include <string.h> // memcpy

#ifdef QUICKSPI_DRIVER_ARDUINO
QuickSPIDevice::QuickSPIDevice(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings): spi(spi), ssPin(ssPin), spiSettings(spiSettings) {}
#elif defined(QUICKSPI_DRIVER_ESPIDF)
QuickSPIDevice::QuickSPIDevice(spi_host_device_t host, gpio_num_t cs_pin, uint32_t clock_speed_hz, uint8_t mode) {
    spi_device_interface_config_t dev_config = { 0 };
    dev_config.clock_speed_hz = clock_speed_hz;
    dev_config.mode = mode;
    dev_config.spics_io_num = cs_pin;
    dev_config.queue_size = 1;
    
    ESP_ERROR_CHECK(spi_bus_add_device(host, &dev_config, &spi_device));
}

QuickSPIDevice::QuickSPIDevice(spi_host_device_t host, spi_device_interface_config_t* device) {
    ESP_ERROR_CHECK(spi_bus_add_device(host, device, &spi_device));
}

#endif

void QuickSPIDevice::writeRegister(uint8_t registerAddress, const uint8_t* buf, size_t len) {
    // Prepare buffer with register address + data
    uint8_t* txbuf = new uint8_t[len + 1];
    txbuf[0] = registerAddress;
    memcpy(txbuf + 1, buf, len);
    
    #ifdef QUICKSPI_DEBUG_WRITES
    Serial.printf("QuickSPI write of size 1+%d of register %02x\r\n", len, registerAddress);
    #endif
    
    // Use raw data method
    writeRawData(txbuf, len + 1);
    
    // Cleanup
    delete[] txbuf;
}

void QuickSPIDevice::writeRawData(const uint8_t* txbuf, size_t len) {
    
    #ifdef QUICKSPI_DEBUG_WRITES
    Serial.printf("QuickSPI raw write of size %d\r\n", len);
    for (size_t i = 0; i < len; i++)
    {
        Serial.printf(" -- TX byte %d: %02x\r\n", i, trxbuf[i]);
    }
    #endif
    
    // SPI transaction
#ifdef QUICKSPI_DRIVER_ARDUINO
    // Allocate buffer for transmission
    uint8_t* trxbuf = new uint8_t[len];
    // Copy source data
    memcpy(trxbuf, txbuf, len);

    spi.beginTransaction(spiSettings);
    digitalWrite(ssPin, LOW);
    spi.transfer(trxbuf, len);
    digitalWrite(ssPin, HIGH);
    spi.endTransaction();

    // Cleanup
    delete[] trxbuf;
#elif defined(QUICKSPI_DRIVER_ESPIDF)
    // Transfer directly using txbuf
    spi_transaction_t trans = {};
    trans.length = len * 8; // length in bits
    trans.tx_buffer = txbuf;
    ESP_ERROR_CHECK(spi_device_transmit(spi_device, &trans));
#endif
    
}

void QuickSPIDevice::writeReadRawData(uint8_t* trxbuf, size_t txlen, size_t rxlen) {
    
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
    trans.length = (txlen + rxlen) * 8; // length in bits
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

void QuickSPIDevice::writeAndReadRegister(uint8_t registerAddress, uint8_t* buf, size_t len) {
    // Prepare buffer with register address + data
    uint8_t* trxbuf = new uint8_t[len + 1];
    trxbuf[0] = registerAddress;
    memcpy(trxbuf + 1, buf, len);
    
    #if defined(QUICKSPI_DEBUG_WRITES) || defined(QUICKSPI_DEBUG_READS)
    Serial.printf("QuickSPI read/write of size 1+%d of register %02x\r\n", len, registerAddress);
    #endif
    
    // Use raw data method
    writeReadRawData(trxbuf, len + 1, len + 1);
    
    // Copy received data (skip first byte which is the register address response)
    memcpy(buf, trxbuf + 1, len);
    
    // Cleanup
    delete[] trxbuf;
}

void QuickSPIDevice::readRegister(uint8_t registerAddress, uint8_t* buf, size_t len) {
    // Prepare buffer with register address
    uint8_t* trxbuf = new uint8_t[len + 1];
    trxbuf[0] = registerAddress;
    
    #ifdef QUICKSPI_DEBUG_READS
    Serial.printf("QuickSPI read of size 1+%d of register %02x\r\n", len, registerAddress);
    #endif
    
    // Use raw data method (send 1 byte, receive len+1 bytes)
    writeReadRawData(trxbuf, 1, len + 1);
    
    // Copy received data (skip first byte which is the register address response)
    memcpy(buf, trxbuf + 1, len);
    
    // Cleanup
    delete[] trxbuf;
}

bool QuickSPIDevice::writeAndVerifyData(uint8_t readAddress, uint8_t writeAddress, const uint8_t* buf, size_t len) {
    // Try to write - if it fails, do not try to verify
    writeRegister(writeAddress, buf, len);
    // Insert grace time between write and read
#ifdef QUICKSPI_DRIVER_ARDUINO
    delay(delayBetweenWriteAndRead);
#elif defined(QUICKSPI_DRIVER_ESPIDF)
    vTaskDelay(pdMS_TO_TICKS(delayBetweenWriteAndRead));
#endif
    // Read back data for verify
    uint8_t* rxbuf = new uint8_t[len];
    readRegister(readAddress, rxbuf, len);
    // Compare data
    bool result = memcmp(rxbuf, buf, len) == 0; // true => rx data is the same as tx data
    // cleanup
    delete[] rxbuf;
    return result;
}

uint8_t QuickSPIDevice::read8BitRegister(uint8_t registerAddress) {
    uint8_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 1);
    return ret;
}

uint16_t QuickSPIDevice::read16BitRegister(uint8_t registerAddress) {
    uint16_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 2);
    return ret;
}

uint32_t QuickSPIDevice::read24BitRegister(uint8_t registerAddress) {
    uint32_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 3);
    return ret;
}

uint32_t QuickSPIDevice::read32BitRegister(uint8_t registerAddress) {
    uint32_t ret = 0;
    readRegister(registerAddress, (uint8_t*)&ret, 4);
    return ret;
}

void QuickSPIDevice::write8BitRegister(uint8_t registerAddress, uint8_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 1);
}

void QuickSPIDevice::write16BitRegister(uint8_t registerAddress, uint16_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 2);
}

void QuickSPIDevice::write24BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 3);
}

void QuickSPIDevice::write32BitRegister(uint8_t registerAddress, uint32_t value) {
    return writeRegister(registerAddress, (uint8_t*)&value, 4);
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
