#include <SPI.h>
#include <QuickSPI.h>

class SmokeDevice : public QuickSPIDevice {
public:
    SmokeDevice(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings)
        : QuickSPIDevice(spi, ssPin, spiSettings) {}
};

SPIClass smokeSPI(HSPI);
SmokeDevice smokeDevice(smokeSPI, 5, SPISettings(1000000, MSBFIRST, SPI_MODE0));

void setup() {
    (void)smokeDevice;
}

void loop() {
}