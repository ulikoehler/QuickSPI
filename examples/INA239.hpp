#pragma once
#include <QuickSPI.h>

template<
    // Read filters
    Postprocessor8Bit postprocessRead8=noop,
    Postprocessor16Bit postprocessRead16=invertByteorder16,
    Postprocessor24Bit postprocessRead24=invertByteorder24,
    Postprocessor32Bit postprocessRead32=invertByteorder32,
    // Write filters
    Postprocessor8Bit postprocessWrite8=noop,
    Postprocessor16Bit postprocessWrite16=noop,
    Postprocessor24Bit postprocessWrite24=noop,
    Postprocessor32Bit postprocessWrite32=noop
>
class INA239 : public QuickSPIDevice {
public:
    inline INA239(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings): QuickSPIDevice(spi, ssPin, spiSettings) {}

    void FastAllMode() {
        // No averaging with 2.074ms sample time results in fastest
        // value with maximum possible 15.7 ENOB
        writeADCConfig(
            ADCConfig::ModeContinousAll |
            ADCConfig::TempConversionTime2074us |
            ADCConfig::ShuntVoltageConversionTime2074us|
            ADCConfig::BusVoltageConversionTime2074us |
            ADCConfig::Average1 // 64ms total per conversion
        );
    }

    void SlowAllMode() {
        writeADCConfig(
            ADCConfig::ModeContinousAll |
            ADCConfig::TempConversionTime4120us |
            ADCConfig::ShuntVoltageConversionTime4120us |
            ADCConfig::BusVoltageConversionTime4120us |
            ADCConfig::Average16 // 64ms total per conversion
        );
    }

    void Configure() {
        // Current resolution ("Current_LSB") == 305.1758uA, same as reference design of section 8.2.2.3 in the datasheet
        // --> SHUNT_CAL = 819.2e6 * 305.1758e-6 * 16e-3 = 4000.000
        writeShuntCalibration(4000);
        writeConfig(Config::ShuntFullScale163mV);
        SlowAllMode(); // Set default mode
    }

    static constexpr uint8_t ReadFlag = 1;
    static constexpr uint8_t WriteFlag = 0;

    QUICKSPI_DEFINE_REGISTER16_RW(Config, (0x00 << 2) | ReadFlag, (0x00 << 2) | WriteFlag) {
        ShuntFullScale163mV = 0 << 4,
        ShuntFullScale41mV = 1 << 4
    };
    QUICKSPI_DEFINE_REGISTER16_RW(ADCConfig, (0x01 << 2) | ReadFlag, (0x01 << 2) | WriteFlag) {
        Average1 = 0x0 << 0,
        Average4 = 0x1 << 0,
        Average16 = 0x2 << 0,
        Average64 = 0x3 << 0,
        Average128 = 0x4 << 0,
        Average256 = 0x5 << 0,
        Average512 = 0x6 << 0,
        Average1024 = 0x7 << 0,

        TempConversionTime50us = 0x0 << 3, 
        TempConversionTime84us = 0x1 << 3, 
        TempConversionTime150us = 0x2 << 3, 
        TempConversionTime280us = 0x3 << 3, 
        TempConversionTime540us = 0x4 << 3, 
        TempConversionTime1052us = 0x5 << 3, 
        TempConversionTime2074us = 0x6 << 3, 
        TempConversionTime4120us = 0x7 << 3, 

        ShuntVoltageConversionTime50us = 0x0 << 6,
        ShuntVoltageConversionTime84us = 0x1 << 6,
        ShuntVoltageConversionTime150us = 0x2 << 6,
        ShuntVoltageConversionTime280us = 0x3 << 6,
        ShuntVoltageConversionTime540us = 0x4 << 6,
        ShuntVoltageConversionTime1052us = 0x5 << 6,
        ShuntVoltageConversionTime2074us = 0x6 << 6,
        ShuntVoltageConversionTime4120us = 0x7 << 6,

        BusVoltageConversionTime50us = 0x0 << 9,
        BusVoltageConversionTime84us = 0x1 << 9,
        BusVoltageConversionTime150us = 0x2 << 9,
        BusVoltageConversionTime280us = 0x3 << 9,
        BusVoltageConversionTime540us = 0x4 << 9,
        BusVoltageConversionTime1052us = 0x5 << 9,
        BusVoltageConversionTime2074us = 0x6 << 9,
        BusVoltageConversionTime4120us = 0x7 << 9,

        ModeShutdown = 0x0 << 9,
        ModeTriggeredSingleShotBusVoltage = 0x1 << 9,
        ModeTriggeredSingleShotShuntVoltage = 0x2 << 9,
        ModeTriggeredShuntAndBusVoltage = 0x3 << 9,
        ModeTriggeredSingleShotTemperature = 0x4 << 9,
        ModeTriggeredSingleShotTemperatureAndBusVoltage = 0x5 << 9,
        ModeTriggeredSingleShotTemperatureAndShuntVoltage = 0x6 << 9,
        ModeTriggeredAll = 0x7 << 9,
        ModeShutdownAlt = 0x8 << 9, // Alternative value for shutdown
        ModeContinousBusVoltage = 0x9 << 9,
        ModeContinousShuntVoltage = 0xA << 9,
        ModeContinousShuntAndBusVoltage = 0xB << 9,
        ModeContinousTemperature = 0xC << 9,
        ModeContinousBusVoltageAndTemperature = 0xD << 9,
        ModeContinousShuntVoltageAndTemperature = 0xE << 9,
        ModeContinousAll = 0xF << 9
    };
    QUICKSPI_DEFINE_REGISTER16_RW(ShuntCalibration, (0x02 << 2) | ReadFlag, (0x02 << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(VShunt, (0x04 << 2) | ReadFlag, (0x04 << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RO(VBus, (0x05 << 2) | ReadFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RO(DieTemperature, (0x06 << 2) | ReadFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RO(Current, (0x07 << 2) | ReadFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RO(Power, (0x08 << 2) | ReadFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(DiagAlert, (0x0B << 2) | ReadFlag, (0x0B << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(ShuntOvervoltageThreshold, (0x0C << 2) | ReadFlag, (0x0C << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(ShuntUndervoltageThreshold, (0x0D << 2) | ReadFlag, (0x0D << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(BusOvervoltageThreshold, (0x0E << 2) | ReadFlag, (0x0E << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(BusUndervoltageThreshold, (0x0F << 2) | ReadFlag, (0x0F << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(TemperatureOverlimitThreshold, (0x10 << 2) | ReadFlag, (0x10 << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RW(PowerOverlimitThreshold, (0x11 << 2) | ReadFlag, (0x11 << 2) | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RO(ManufacturerID, (0x3E << 2) | ReadFlag) {};
    QUICKSPI_DEFINE_REGISTER16_RO(DeviceID, (0x3F << 2) | ReadFlag) {};


    float ComputeCurrent() {
        return readVShunt() * 5e-6 / 16e-3; // 5 microvolts resolution, 16mOhm shunt
    }

    float ComputeTemperature() {
        constexpr float temperatureResolution = 125e-3;
        return (readDieTemperature() >> 4) * temperatureResolution;
    }

    float ComputeVoltage() {
        return readVBus() * 3.125e-3;
    }
};
