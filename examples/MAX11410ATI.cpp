#include "MAX11410ATI.hpp"

void AllMAX11410RegistersToJson(MAX11410ATI<> adc, JsonObject& registers) {
    registers["PowerDown"] = adc.readPowerDownRegister();
    registers["ConversionStart"] = adc.readConversionStart();
    registers["SequenceStart"] = adc.readSequenceStart();
    registers["CalibrationStart"] = adc.readCalibrationStart();
    registers["GPIO0Control"] = adc.readGPIO0Control();
    registers["GPIO1Control"] = adc.readGPIO1Control();
    registers["GPIOConversion"] = adc.readGPIOConversion();
    registers["GPIOSequencerAddress"] = adc.readGPIOSequencerAddress();
    registers["Filter"] = adc.readFilter();
    registers["Control"] = adc.readControl();
    registers["Source"] = adc.readSource();
    registers["MUXControl0"] = adc.readMUXControl0();
    registers["MUXControl1"] = adc.readMUXControl1();
    registers["MUXControl2"] = adc.readMUXControl2();
    registers["WaitExtension"] = adc.readWaitExtension();
    registers["WaitStart"] = adc.readWaitStart();
    registers["PartID"] = adc.readPartID();
    registers["SystemCalibrationSelect"] = adc.readSystemCalibrationSelect();
    registers["SystemOffsetA"] = adc.readSystemOffsetA();
    registers["SystemOffsetB"] = adc.readSystemOffsetB();
    registers["SystemGainA"] = adc.readSystemGainA();
    registers["SystemGainB"] = adc.readSystemGainB();
    registers["SelfCalibrationOffset"] = adc.readSelfCalibrationOffset();
    registers["SelfCalibrationGain1"] = adc.readSelfCalibrationGain1();
    registers["SelfCalibrationGain2"] = adc.readSelfCalibrationGain2();
    registers["SelfCalibrationGain4"] = adc.readSelfCalibrationGain4();
    registers["SelfCalibrationGain8"] = adc.readSelfCalibrationGain8();
    registers["SelfCalibrationGain16"] = adc.readSelfCalibrationGain16();
    registers["SelfCalibrationGain32"] = adc.readSelfCalibrationGain32();
    registers["SelfCalibrationGain64"] = adc.readSelfCalibrationGain64();
    registers["SelfCalibrationGain128"] = adc.readSelfCalibrationGain128();

    auto lowerThreshold = registers.createNestedArray("LowerThreshold");
    lowerThreshold.add(adc.readLowerThreshold0());
    lowerThreshold.add(adc.readLowerThreshold1());
    lowerThreshold.add(adc.readLowerThreshold2());
    lowerThreshold.add(adc.readLowerThreshold3());
    lowerThreshold.add(adc.readLowerThreshold4());
    lowerThreshold.add(adc.readLowerThreshold5());
    lowerThreshold.add(adc.readLowerThreshold6());
    lowerThreshold.add(adc.readLowerThreshold7());

    auto upperThreshold = registers.createNestedArray("UpperThreshold");
    upperThreshold.add(adc.readUpperThreshold0());
    upperThreshold.add(adc.readUpperThreshold1());
    upperThreshold.add(adc.readUpperThreshold2());
    upperThreshold.add(adc.readUpperThreshold3());
    upperThreshold.add(adc.readUpperThreshold4());
    upperThreshold.add(adc.readUpperThreshold5());
    upperThreshold.add(adc.readUpperThreshold6());
    upperThreshold.add(adc.readUpperThreshold7());

    auto data = registers.createNestedArray("Data");
    data.add(adc.readData0());
    data.add(adc.readData1());
    data.add(adc.readData2());
    data.add(adc.readData3());
    data.add(adc.readData4());
    data.add(adc.readData5());
    data.add(adc.readData6());
    data.add(adc.readData7());

    registers["Status"] = adc.readStatus();
    registers["StatusInterruptEnable"] = adc.readStatusInterruptEnable();
}
