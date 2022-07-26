#pragma once
#include <QuickSPI.h>
#include <ArduinoJson.h>

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
class MAX11410ATI : public QuickSPIDevice {
public:

    inline MAX11410ATI(SPIClass& spi, uint8_t ssPin, SPISettings spiSettings): QuickSPIDevice(spi, ssPin, spiSettings) {
    }

    static constexpr uint8_t ReadFlag = 1 << 7;
    static constexpr uint8_t WriteFlag = 0 << 7;

    bool IsConversionFinished() {
        return (readStatus() & Status::ConversionCompleted) != 0;
    }

    /**
     * Acquire a single ADC conversion value, involving:
     *  - Selecting the channel
     *  - Starting the conversion
     *  - Waiting for the conversion to finish, running taskYIELD() when not finished yet
     * 
     * @param channel 
     * @return uint32_t 
     */
    uint32_t AcquireSingleADCConversion(uint8_t channel) {
        SelectSingleEndedInputChannel(channel);
        // Perform conversion and wait for it to finish
        StartSingleConversion(channel);
        while(!IsConversionFinished()) {
            taskYIELD(); // Wait as little as possible
        }
        // Read data
        return postprocessRead24(0, read24BitRegister(MAX11410ATI::Data0ReadAddress + channel));
        // bswap: data is returned in big endian format
        // >>8: bswap shifts 0x00 byte (it's a 24 bit data register!) to the LSB.
        //return __builtin_bswap32(data) >> 8;
    }

    void AutoCalibrateADC() {
        writeCalibrationStart(0b000); // Start no-PGA self calibration
        while(!IsCalibrationFinished()) {
            taskYIELD(); // Wait as little as possible
        }
    }

    bool IsCalibrationFinished() {
        return (readStatus() & Status::CalibrationCompleted) != 0;
    }

    uint32_t AcquireADCCalibrationValue(uint8_t channel) {
        // Auto-calibrate first to ensure the utmost precision possible
        AutoCalibrateADC();
        // Average of 16 samples
        uint64_t sum = AcquireSummedADCConversion(channel, 16);
        return sum >> 4; // 2**4 == 16
    }

    void ConfigureADC() {
        writePowerDownRegister(0); // Normal mode - not power down
        writeControl(
            Control::ReferenceREF1P_AGND |
            Control::UnipolarInput
        );
        writeFilter(
            MAX11410ATI::Filter::FIR50Hz  
         | 0b0101 /* 35.6 Hz single conversion, 40 Hz continous. Fastest possible rate with FIR50Hz */);
        AutoCalibrateADC();
    }

    /**
     * @brief Acquire a sum of [navg] conversions. This is typically used for averaging
     * 
     * 
     * @param adc 
     * @param channel 
     * @param navg 
     * @return uint32_t 
     */
    uint64_t AcquireSummedADCConversion(uint8_t channel, size_t navg) {
        SelectSingleEndedInputChannel(channel);
        // Perform conversion and wait for it to finish
        uint64_t sum = 0;
        for (size_t i = 0; i < navg; i++)
        {
            StartSingleConversion(channel);
            while(!IsConversionFinished()) {
                taskYIELD(); // Wait as little as possible
            }
            // Read data
            sum += postprocessRead24(0, read24BitRegister(MAX11410ATI::Data0ReadAddress + channel));
        }
        return sum;
    }

    void SelectSingleEndedInputChannel(uint8_t channel) {
        writeMUXControl0((MUXControl0::AINP_AIN1 * channel) | MUXControl0::AINN_AGND);
    }

    void StartSingleConversion(uint8_t dataRegisterIndex) {
        writeConversionStart(ConversionStart::ConversionType_Single | (ConversionStart::Destination_Data1 * dataRegisterIndex));
    }

    QUICKSPI_DEFINE_REGISTER8_RW(PowerDownRegister, 0x00 | ReadFlag, 0x00 | WriteFlag) {
        Normal = 0b00,
        Standby = 0b01,
        Sleep = 0b10,
        Reset = 0b11
    };
    QUICKSPI_DEFINE_REGISTER8_RW(ConversionStart, 0x01 | ReadFlag, 0x01 | WriteFlag) {
        ConversionType_Single = 0b00 << 0,
        ConversionType_Continous = 0b01 << 0,
        Destination_Data0 = 0b000 << 4,
        Destination_Data1 = 0b001 << 4,
        Destination_Data2 = 0b010 << 4,
        Destination_Data3 = 0b011 << 4,
        Destination_Data4 = 0b100 << 4,
        Destination_Data5 = 0b101 << 4,
        Destination_Data6 = 0b110 << 4,
        Destination_Data7 = 0b111 << 4
    };
    QUICKSPI_DEFINE_REGISTER8_RW(SequenceStart, 0x02 | ReadFlag, 0x02 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(CalibrationStart, 0x03 | ReadFlag, 0x03 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(GPIO0Control, 0x04 | ReadFlag, 0x04 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(GPIO1Control, 0x05 | ReadFlag, 0x05 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(GPIOConversion, 0x06 | ReadFlag, 0x06 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(GPIOSequencerAddress, 0x07 | ReadFlag, 0x07 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(Filter, 0x08 | ReadFlag, 0x08 | WriteFlag) {
        Both5060Hz = 0b00 << 4,
        FIR50Hz = 0b01 << 4,
        FIR60Hz = 0b10 << 4,
        SINC = 0b11 << 4,
        // RATE not specified here, just OR!        
    };
    QUICKSPI_DEFINE_REGISTER8_RW(Control, 0x09 | ReadFlag, 0x09 | WriteFlag) {
        ExternalClock = 1 << 7,
        UnipolarInput = 1 << 6,
        OffsetBinaryFormat = 1 << 5,
        PositiveReferenceBufferEnable = 1 << 4,
        NegativeReferenceBufferEnable = 1 << 3,
        ReferenceAIN0_1 = 0b000 << 0,
        ReferenceREF1PN = 0b001 << 0,
        ReferenceREF2PN = 0b010 << 0,
        ReferenceAVDD_GND  = 0b011 << 0,
        ReferenceAIN0_GND = 0b100 << 0,
        ReferenceREF1P_AGND = 0b101 << 0,
        ReferenceREF2P_AGND = 0b110 << 0,
        ReferenceAVDD_GND_ALT = 0b111 << 0
    };
    QUICKSPI_DEFINE_REGISTER8_RW(Source, 0x0A | ReadFlag, 0x0A | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(MUXControl0, 0x0B | ReadFlag, 0x0B | WriteFlag) {
        // Analog positive input definitions
        AINP_AIN0 = 0b0000 << 4,
        AINP_AIN1 = 0b0001 << 4,
        AINP_AIN2 = 0b0010 << 4,
        AINP_AIN3 = 0b0011 << 4,
        AINP_AIN4 = 0b0100 << 4,
        AINP_AIN5 = 0b0101 << 4,
        AINP_AIN6 = 0b0110 << 4,
        AINP_AIN7 = 0b0111 << 4,
        AINP_AIN8 = 0b1000 << 4,
        AINP_AIN9 = 0b1001 << 4,
        AINP_AVDD = 0b1010 << 4,
        // Analog negative input
        AINN_AIN0 = 0b0000 << 0,
        AINN_AIN1 = 0b0001 << 0,
        AINN_AIN2 = 0b0010 << 0,
        AINN_AIN3 = 0b0011 << 0,
        AINN_AIN4 = 0b0100 << 0,
        AINN_AIN5 = 0b0101 << 0,
        AINN_AIN6 = 0b0110 << 0,
        AINN_AIN7 = 0b0111 << 0,
        AINN_AIN8 = 0b1000 << 0,
        AINN_AIN9 = 0b1001 << 0,
        AINN_AGND = 0b1010 << 0
    };
    QUICKSPI_DEFINE_REGISTER8_RW(MUXControl1, 0x0C | ReadFlag, 0x0C | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(MUXControl2, 0x0D | ReadFlag, 0x0D | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(WaitExtension, 0x0F | ReadFlag, 0x0F | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER8_RW(WaitStart, 0x10 | ReadFlag, 0x10 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RO(PartID, 0x11 | ReadFlag) {};
    // System calibration
    QUICKSPI_DEFINE_REGISTER24_RW(SystemCalibrationSelect, 0x12 | ReadFlag, 0x12 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SystemOffsetA, 0x13 | ReadFlag, 0x13 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SystemOffsetB, 0x14 | ReadFlag, 0x14 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SystemGainA, 0x15 | ReadFlag, 0x15 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SystemGainB, 0x16 | ReadFlag, 0x16 | WriteFlag) {};
    // Self calibration
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationOffset, 0x17 | ReadFlag, 0x17 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain1, 0x18 | ReadFlag, 0x18 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain2, 0x19 | ReadFlag, 0x19 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain4, 0x1A | ReadFlag, 0x1A | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain8, 0x1B | ReadFlag, 0x1B | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain16, 0x1C | ReadFlag, 0x1C | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain32, 0x1D | ReadFlag, 0x1D | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain64, 0x1E | ReadFlag, 0x1E | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(SelfCalibrationGain128, 0x1F | ReadFlag, 0x1F | WriteFlag) {};
    // Lower threshold
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold0, 0x20 | ReadFlag, 0x20 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold1, 0x21 | ReadFlag, 0x21 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold2, 0x22 | ReadFlag, 0x22 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold3, 0x23 | ReadFlag, 0x23 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold4, 0x24 | ReadFlag, 0x24 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold5, 0x25 | ReadFlag, 0x25 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold6, 0x26 | ReadFlag, 0x26 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(LowerThreshold7, 0x27 | ReadFlag, 0x27 | WriteFlag) {};
    // Upper threshold
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold0, 0x28 | ReadFlag, 0x28 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold1, 0x29 | ReadFlag, 0x29 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold2, 0x2A | ReadFlag, 0x2A | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold3, 0x2B | ReadFlag, 0x2B | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold4, 0x2C | ReadFlag, 0x2C | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold5, 0x2D | ReadFlag, 0x2D | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold6, 0x2E | ReadFlag, 0x2E | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(UpperThreshold7, 0x2F | ReadFlag, 0x2F | WriteFlag) {};
    // Conversion data registers
    QUICKSPI_DEFINE_REGISTER24_RW(Data0, 0x30 | ReadFlag, 0x30 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(Data1, 0x31 | ReadFlag, 0x31 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(Data2, 0x32 | ReadFlag, 0x32 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(Data3, 0x33 | ReadFlag, 0x33 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(Data4, 0x34 | ReadFlag, 0x34 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(Data5, 0x35 | ReadFlag, 0x35 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(Data6, 0x36 | ReadFlag, 0x36 | WriteFlag) {};
    QUICKSPI_DEFINE_REGISTER24_RW(Data7, 0x37 | ReadFlag, 0x37 | WriteFlag) {};
    // Status & interrupt registers
    QUICKSPI_DEFINE_REGISTER24_RW(Status, 0x38 | ReadFlag, 0x38 | WriteFlag) {
        ConversionCompleted = 1 << 0,
        SequenceCompleted = 1 << 1,
        CalibrationCompleted = 1 << 2,
        WaitComplete = 1 << 3,
        DataReady = 1 << 4,
        SystemGainCalOverrage = 1 << 7,
        ThresholdUnderrange0 = 1 << 8,
        ThresholdUnderrange1 = 1 << 9,
        ThresholdUnderrange2 = 1 << 10,
        ThresholdUnderrange3 = 1 << 11,
        ThresholdUnderrange4 = 1 << 12,
        ThresholdUnderrange5 = 1 << 13,
        ThresholdUnderrange6 = 1 << 14,
        ThresholdUnderrange7 = 1 << 15,
        ThresholdOverrange0 = 1 << 16,
        ThresholdOverrange1 = 1 << 17,
        ThresholdOverrange2 = 1 << 18,
        ThresholdOverrange3 = 1 << 19,
        ThresholdOverrange4 = 1 << 20,
        ThresholdOverrange5 = 1 << 21,
        ThresholdOverrange6 = 1 << 22,
        ThresholdOverrange7 = 1 << 23
    };
    QUICKSPI_DEFINE_REGISTER24_RW(StatusInterruptEnable, 0x39 | ReadFlag, 0x39 | WriteFlag) {};
};

void AllMAX11410RegistersToJson(MAX11410ATI<> adc, JsonObject& registers);