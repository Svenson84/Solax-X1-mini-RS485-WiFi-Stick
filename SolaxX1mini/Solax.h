#ifndef __SOLAX_H__
#define __SOLAX_H__

#include <Arduino.h>

#define SOLAX_RS485_BAURATE           9600  // bps
#define SOLAX_RS485_INVERTER_DELAY    500   // ms (max)
#define SOLAX_RS485_INTER_CHAR_DELAY  200   // ms (max)
#define SOLAX_RS485_INTER_FRAME_DELAY 500   // ms (min)
#define SOLAX_RS485_REQUEST_CYCLE     1500  // ms
#define SOLAX_RS485_INVERTER_TIMEOUT  30    // sec
#define SOLAX_BROADCAST_ADDRESS 0xFF    // TODO: is that needed?
#define SOLAX_GATEWAY_ADDRESS         0x01  // (default 0x01)
#define SOLAX_INVERTER_ADDRESS        0x0A  // (default 0x0A) only one inverter on the bus supported at the moment
#define SOLAX_DATA_BYTES              100   // data bytes for RX/TX buffers (+9 bytes will be used for header, address, etc)
#define SOLAX_CTLCODE_REGISTER        0x10
#define SOLAX_CTLCODE_READ            0x11
#define SOLAX_CTLCODE_WRITE           0x12
#define SOLAX_CTLCODE_EXECUTE         0x13

class Solax
{
  private:
    struct SolaxMessageT {
      uint8_t Header[2];
      uint8_t Source[2];
      uint8_t Destination[2];
      uint8_t ControlCode;
      uint8_t FunctionCode;
      uint8_t DataLength;
      uint8_t Data[SOLAX_DATA_BYTES];
      uint8_t Checksum[2];
    };

    struct SolaxDeviceT {
      uint8_t Active;
      uint8_t Address;
      uint8_t SerialNumber[14];
      uint16_t Temperature;     // 1 °C ??
      uint16_t YieldToday;      // 0.1 kWh
      uint16_t PV1Voltage;      // 0.1 V
      uint16_t PV2Voltage;      // 0.1 V
      uint16_t PV1Current;      // 0.1 A
      uint16_t PV2Current;      // 0.1 A
      uint16_t CurrentOutput;   // 0.1 A
      uint16_t GridVoltage;     // 0.1 V
      uint16_t GridFrequency;   // 0.01 Hz
      uint16_t PowerOutput;     // 1 W (Acc. documentation REF{1} it should be 0.1 kW/h, but that is wrong!) 
      uint16_t AvgPowerOutput;  // 1 W (Added to see average power output value
      uint16_t MaxPowerOutput;  // 1 W (Added to see maximum power output value 
      uint16_t NotUsed;
      uint32_t YieldTotal;      // 0.1 kWh
      uint32_t RuntimeTotal;    // 1 h
      uint16_t InverterMode;    // 0: Wait Mode, 1: Normal Mode, 2: Fault Mode, 3: Permanent Fault Mode
      uint16_t GridVoltageFaultValue;   // 0.1 V
      uint16_t GridFrequencyFaultValue; // 0.01 Hz
      uint16_t DcInjectionFaultValue;   // 1 mA
      uint16_t TemperatureFaultValue;   // 1 °C ??
      uint16_t PV1VoltageFaultValue;    // 0.1 V
      uint16_t PV2VoltageFaultValue;    // 0.1 V
      uint16_t GFCFaultValue;           // 1 mA
      uint32_t ErrorMessage;
    };
    
    struct SolaxIDInfoT {
      uint8_t NotUsed[9];
      uint8_t Phases;
      char RatedPower[6];
      char FirmwareVer[5];
      char ModuleName[14];
      char FactoryName[14];
      char SerialNumber[14];
      char RatedBusVoltage[4];
    };

    struct SolaxMessageT RxBuffer;
    uint8_t TxEnablePin;
    uint8_t RxByteCnt;

    bool GetMsg(uint8_t byte);
    bool ProcessRxMsg(void);
    void RequestNormalData(uint8_t Address);
    void RequestIdData(uint8_t Address);
    void RegisterAddress(uint8_t SerialNumber[14], uint8_t Address);
    void DiscoverDevice(void);
    void SendTxMsg(struct SolaxMessageT *Buffer);
    uint16_t CalcChecksum(const struct SolaxMessageT Buffer);
    uint16_t CalcAvgPower(uint16_t ActPower);


  public:
    struct SolaxDeviceT Data;
    struct SolaxIDInfoT Info;
    
    uint16_t Stat_Idx = 0;
    uint16_t Stat_AvgPower[144] {0};
    uint16_t Stat_PwrToday[144] {0};
    
    Solax(uint8_t pin);
    void begin();
    void loop(void);
    void ClearRxBuffer(void);
    void SendDebugMsg(uint8_t DebugData[64]);
    
    
};

#endif /* __SOLAX_H__ */