# QuickSPI

Practical Arduino SPI wrapper, allowing direct register access with proper error handling.

This is intended to be similar to [AdaFruit BusIO](https://github.com/adafruit/Adafruit_BusIO) but provide a more convenient API and less memory footprint. For example, every [Adafruit_BusIO_Register](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_BusIO_Register.h) is an actual object occupying a bunch of bytes of RAM and does not provide verified read functionality.

## How?

QuickSPI provides classes & macro tricks to automatically define appropriate functions for any register, with zero memory footprint for additional registers.

## Arduino INA239 example

This example initializes and reads some registers of the [INA239](https://www.ti.com/product/INA239) voltage & current sense amplifier

```cpp
#include <SPI.h>
#include <QuickSPI.h>

#define Pin_SPI_nCS_ADC 8
SPIClass mySPI(HSPI);

INA239<> ina(mySPI, Pin_SPI_nCS_ADC, SPISettings(1000000, SPI_MSBFIRST, SPI_MODE1));

void setup() {
    ina.Configure();
}

void loop() {
    // Example of automatically generated register read method
    // This method is automatically generated from
    //    QUICKSPI_DEFINE_REGISTER16_RO(ManufacturerID, (0x3E << 2) | ReadFlag) {};
    uint32_t manufacturerId = zoneADC.readManufacturerID(); // Should be 0x5449
    
    // Example 
    float current = zoneADC.ComputeCurrent();
    float voltage = zoneADC.ComputeVoltage();
    
    // TODO Your code goes here...
}

```
