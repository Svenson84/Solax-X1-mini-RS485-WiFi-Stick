#include <Arduino.h>
#include "Solax.h"

Solax::Solax(uint8_t pin)
{
  TxEnablePin = pin;
  RxByteCnt = 0;
  
  RxBuffer = {0};
  Data = {0};
  
  Data.Address = SOLAX_INVERTER_ADDRESS;
  Data.Active = false;
}

void Solax::begin(void) {
  // Configure Serial Port for communication with Solax X1 Mini via RS485
  Serial.setTimeout(1000);  
  Serial.begin(9600, SERIAL_8N1); // SERIAL_8N1 (default) --> 8 data bits, No parity, 1 stop bit

  // Configure GPIO Pin for activation of RS485 transmitter
  pinMode(TxEnablePin, OUTPUT);
  digitalWrite(TxEnablePin, LOW); 
}

void Solax::loop(void) {
  const uint32_t now = millis();
  static uint32_t LastRequest = 0;
  static uint32_t LastRxByte = 0;
  static uint8_t request_toggle = 0;
  
  if((now - LastRxByte) > SOLAX_RS485_INTER_CHAR_DELAY) {
    ClearRxBuffer();
  }

  if((now - LastRxByte) > (SOLAX_RS485_INVERTER_TIMEOUT * 1000U)) {
    Data.Active = false;
  }

  // Send Request to Inverter (Tx Message)
  if((now - LastRequest) > SOLAX_RS485_REQUEST_CYCLE) {
    switch(request_toggle) {
      case 0:
        DiscoverDevice();
        break;
        
      case 1:
        RequestNormalData(Data.Address);
        break;
        
      case 2:
        //RequestIdData(Data.Address); // TODO: this is not really working 
        break;

      default:
        break;
    }
    
    if(request_toggle >= 2) {
      request_toggle = 0;
    }
    else {
      request_toggle++;
    }

    LastRequest = now;
  }

  delay(5);
  
  // Read Response from Inverter (Rx Message)
  while (Serial.available()) {
    uint8_t byte;
    byte = Serial.read();
    LastRxByte = millis();
    if (GetMsg(byte)) {
      /*String text = "[ModbusSolaxLoop] New RX msg complete: " + String(RxBuffer.Header[1], HEX) + " " + String(RxBuffer.Header[0], HEX) + " " + String(RxBuffer.Source[1], HEX) + " " + String(RxBuffer.Source[0], HEX) + " " + String(RxBuffer.Destination[1], HEX) + " " + String(RxBuffer.Destination[0], HEX) + " " + String(RxBuffer.ControlCode, HEX) + " " + String(RxBuffer.FunctionCode, HEX) + " " + String(RxBuffer.DataLength, HEX);
      for(int i = 0; i<RxBuffer.DataLength; i++) {
        text += " " + String(RxBuffer.Data[i], HEX);
      }
      text += " " + String(RxBuffer.Checksum[1], HEX);
      text += " " + String(RxBuffer.Checksum[0], HEX);
      //Debug(text);
      */
      
      if(ProcessRxMsg()) {
        //Debug("[ModbusSolaxLoop] RX msg processed");

        //Send timestamp of last received byte via RS485 DebugMsg
        //uint8_t data[64];
        //data[0] = (uint8_t) (LastRxByte >> 24) & 0xff;
        //data[1] = (uint8_t) (LastRxByte >> 16) & 0xff;
        //data[2] = (uint8_t) (LastRxByte >> 8) & 0xff;
        //data[3] = (uint8_t) (LastRxByte) & 0xff;
        //SolaxTxMsg_Debug(data);
      }
    } 
    else {
      //ClearRxBuffer();
      //Debug("[RS485] Read Byte: "+ String(byte, HEX) + ", " + String(RxByteCnt));
    }
  }
}


void Solax::ClearRxBuffer(void) {
  RxByteCnt = 0;
}

// Function: GetMsg()
//  Stores received byte from RS485 in the RX Buffer.
// Parameter:
//  uint8_t byte: Provide new byte from RS485 interface.
// Return:
//  bool: TRUE if RS485 message from header to checksum was completely received.
//        FALSE if RS485 message is still incomplete or new byte was invalid.
bool Solax::GetMsg(uint8_t byte) {
  RxByteCnt++;
  
  // Byte 1: Header MSB (0xAA)
  if(RxByteCnt == 1) {
    if(byte == 0xAA) {
      RxBuffer.Header[1] = byte;
    }
    else {
      //Debug("[SolaxGetMsg] Invalid Header MSB: 0x" + String(byte, HEX));
      ClearRxBuffer();
    }
    return(false);
  }
  
  // Byte 2: Header LSB (0x55)
  if(RxByteCnt == 2) {
    if(byte == 0x55) {
      RxBuffer.Header[0] = byte;
    }
    else {
      //Debug("[SolaxGetMsg] Invalid Header LSB: 0x" + String(byte, HEX));
      ClearRxBuffer();
    }
    return(false);
  }
  
  // Byte 3: Source Address MSB
  // acc. REF{1}, table in chapter 4: Source Address MSB should be zero for packets from the inverter
  if(RxByteCnt == 3) {
    if(byte == 0x00) {
      RxBuffer.Source[1] = byte;
    }
    else {
      //Debug("[SolaxGetMsg] Invalid Source Address MSB: 0x" + String(byte, HEX));
      ClearRxBuffer();
    }
    return(false);
  }

  // Byte 4: Source Address LSB 
  // acc. REF{1}, table in chapter 4: Source Address LSB should contain the Inverter Address for incoming packets 
  // For unregistered inverters the value is 0x00 or 0xff ??
  if(RxByteCnt == 4) {
    RxBuffer.Source[0] = byte;
    return(false);
  }
  
  // Byte 5: Destination Address MSB
  // acc. REF{1}, table in chapter 4: Destination Address MSB should contain Gateway Adress (AP) for incoming packets 
  if(RxByteCnt == 5) {
    if(byte == SOLAX_GATEWAY_ADDRESS) {
      RxBuffer.Destination[1] = byte;
    }
    else {
      //Debug("[SolaxGetMsg] Invalid Destination Address MSB: 0x" + String(byte, HEX));
      ClearRxBuffer();
    }
    return(false);
  }

  // Byte 6: Destination Address LSB
  // acc. REF{1}, table in chapter 4: Destination Address LSB should be zero for incoming packets 
  if(RxByteCnt == 6) {
    if(byte == 0x00) { 
      RxBuffer.Destination[0] = byte;
    }
    else {
      //Debug("[SolaxGetMsg] Invalid Destination Address LSB: 0x" + String(byte, HEX));
      ClearRxBuffer();
    }
    return(false);
  }

  // Byte 7: Control Byte
  if(RxByteCnt == 7) {
    RxBuffer.ControlCode = byte;
    return(false);
  }
  
  // Byte 8: Function Code
  if(RxByteCnt == 8) {
    RxBuffer.FunctionCode = byte;
    return(false);
  }
  
  // Byte 9: Data Length (N)
  if(RxByteCnt == 9) {
    RxBuffer.DataLength = byte;
    return(false);
  }

  // Byte 10...(10+N-1): Data Bytes
  if(RxByteCnt >= 10 && RxByteCnt <= (10 + RxBuffer.DataLength - 1)) {
    RxBuffer.Data[RxByteCnt - 10] = byte;
    return(false);
  }
  
  // Byte 10+N: Checksum MSB
  if(RxByteCnt == (10 + RxBuffer.DataLength)) {
    RxBuffer.Checksum[1] = byte;
    return(false);
  }

  // Byte 10+N+1: Checksum LSB
  if(RxByteCnt == (10 + RxBuffer.DataLength + 1)) {
    // Message completely received!
    RxBuffer.Checksum[0] = byte; 

    // Check Checksum
    uint16_t ReceivedChecksum = (uint16_t) RxBuffer.Checksum[1] << 8 | (uint16_t) RxBuffer.Checksum[0];
    uint16_t CalculatedChecksum = CalcChecksum(RxBuffer);
    if (ReceivedChecksum != CalculatedChecksum) {
      //Debug("[SolaxGetMsg] Incorrect Checksum: 0x" + String(ReceivedChecksum, HEX) + " (Expected: 0x" + String(CalculatedChecksum, HEX) + ")");
      return(false);
    }
  }
  else {
    //Debug("[SolaxGetMsg] Byte read invalid");
    ClearRxBuffer();
    return(false);
  }

  return(true);
}

// Function: ProcessRxMsg()
//  Processes a completely received message.
// Parameter:
//  const struct SolaxMessageT: Provide new message to process.
// Return:
//  bool: TRUE if message was successful processed.
//        FALSE if an error occured.
bool Solax::ProcessRxMsg(void) { //const struct SolaxMessageT RxMsg) {
  bool ReturnValue = false;

  if(RxBuffer.Source[0] != SOLAX_INVERTER_ADDRESS) {
    //Debug("[ProcessRxMsg] Rx Message from unknown Device: 0x" + String(RxBuffer.Source[0], HEX));
  }
  switch(RxBuffer.ControlCode) {
    case SOLAX_CTLCODE_REGISTER: // Control Code = 0x10 'register'
      // Function Code = 0x80: Register Request from Inverter
      if(RxBuffer.FunctionCode == 0x80 && 
         RxBuffer.DataLength == 0x0E) {
        memcpy(Data.SerialNumber, RxBuffer.Data, 0x0E);
        //Debug("[ProcessRxMsg] Found new Device with Serial Number: " + String(Data.SerialNumber[0], HEX) + " " + String(Data.SerialNumber[1], HEX) + " " + String(Data.SerialNumber[2], HEX) + " " + String(Data.SerialNumber[3], HEX) + " " + String(Data.SerialNumber[4], HEX) + " " + String(Data.SerialNumber[5], HEX) + " " + String(Data.SerialNumber[6], HEX) + " " + String(Data.SerialNumber[7], HEX) + " " + String(Data.SerialNumber[8], HEX) + " " + String(Data.SerialNumber[9], HEX) + " " + String(Data.SerialNumber[10], HEX) + " " + String(Data.SerialNumber[11], HEX) + " " + String(Data.SerialNumber[12], HEX) + " " + String(Data.SerialNumber[13], HEX));
        
        RegisterAddress(Data.SerialNumber, SOLAX_INVERTER_ADDRESS);
        
        ReturnValue = true;
      }
      // Function Code = 0x81: Address Confirm from Inverter
      if(RxBuffer.FunctionCode == 0x81 &&
         RxBuffer.Source[0] == SOLAX_INVERTER_ADDRESS &&
         RxBuffer.DataLength == 0x01 &&
         RxBuffer.Data[0] == 0x06) {
        //Debug("[ProcessRxMsg] Inverter address confirm received");
        Data.Address = RxBuffer.Source[0];
      }
      
      // Function Code = 0x82: Remove confirm from Inverter
      // TODO
      //if(RxBuffer.FunctionCode == 0x82) {
      // }
      //break;
      
    case SOLAX_CTLCODE_READ: // Control Code = 0x11 'read'
      // Function Code = 0x82: Response for "normal info"
      if(RxBuffer.FunctionCode == 0x82 &&
         RxBuffer.Source[0] == SOLAX_INVERTER_ADDRESS) {
        // memcpy leads to wrong byte order for uint16_t/uint32_t
        //memcpy(&Data.Temperature, &RxBuffer.Data[0], RxBuffer.DataLength);
        Data.Temperature = (RxBuffer.Data[0] << 8) | RxBuffer.Data[1];
        Data.YieldToday = (RxBuffer.Data[2] << 8) | RxBuffer.Data[3];
        Data.PV1Voltage = (RxBuffer.Data[4] << 8) | RxBuffer.Data[5];
        Data.PV2Voltage = (RxBuffer.Data[6] << 8) | RxBuffer.Data[7];
        Data.PV1Current = (RxBuffer.Data[8] << 8) | RxBuffer.Data[9];
        Data.PV2Current = (RxBuffer.Data[10] << 8) | RxBuffer.Data[11];
        Data.CurrentOutput = (RxBuffer.Data[12] << 8) | RxBuffer.Data[13];
        Data.GridVoltage = (RxBuffer.Data[14] << 8) | RxBuffer.Data[15];
        Data.GridFrequency = (RxBuffer.Data[16] << 8) | RxBuffer.Data[17];
        Data.PowerOutput = (RxBuffer.Data[18] << 8) | RxBuffer.Data[19];
        Data.NotUsed = (RxBuffer.Data[20] << 8) | RxBuffer.Data[21];
        Data.YieldTotal = (RxBuffer.Data[22] << 24) | (RxBuffer.Data[23] << 16) | (RxBuffer.Data[24] << 8) | RxBuffer.Data[25];
        Data.RuntimeTotal = (RxBuffer.Data[26] << 24) | (RxBuffer.Data[27] << 16) | (RxBuffer.Data[28] << 8) | RxBuffer.Data[29];
        Data.InverterMode = (RxBuffer.Data[30] << 8) | RxBuffer.Data[31];
        Data.GridVoltageFaultValue = (RxBuffer.Data[32] << 8) | RxBuffer.Data[33];
        Data.GridFrequencyFaultValue = (RxBuffer.Data[34] << 8) | RxBuffer.Data[35];
        Data.DcInjectionFaultValue = (RxBuffer.Data[36] << 8) | RxBuffer.Data[37];
        Data.TemperatureFaultValue = (RxBuffer.Data[38] << 8) | RxBuffer.Data[39];
        Data.PV1VoltageFaultValue = (RxBuffer.Data[40] << 8) | RxBuffer.Data[41];
        Data.PV2VoltageFaultValue = (RxBuffer.Data[42] << 8) | RxBuffer.Data[43];
        Data.GFCFaultValue = (RxBuffer.Data[44] << 8) | RxBuffer.Data[45];
        Data.ErrorMessage = (RxBuffer.Data[46] << 24) | (RxBuffer.Data[47] << 16) | (RxBuffer.Data[48] << 8) | RxBuffer.Data[49];

        if(Data.PowerOutput > Data.MaxPowerOutput) Data.MaxPowerOutput = Data.PowerOutput;
        Data.AvgPowerOutput = CalcAvgPower(Data.PowerOutput);
        
        Data.Active = true;
        ReturnValue = true;
      }

      // Function Code = 0x83: Response for "ID info"
      if(RxBuffer.FunctionCode == 0x83 &&
         RxBuffer.Source[0] == SOLAX_INVERTER_ADDRESS) {
        Info.NotUsed[0] = RxBuffer.Data[0];
        Info.NotUsed[1] = RxBuffer.Data[1];
        Info.NotUsed[2] = RxBuffer.Data[2];
        Info.NotUsed[3] = RxBuffer.Data[3];
        Info.NotUsed[4] = RxBuffer.Data[4];
        Info.NotUsed[5] = RxBuffer.Data[5];
        Info.NotUsed[6] = RxBuffer.Data[6];
        Info.NotUsed[7] = RxBuffer.Data[7];
        Info.NotUsed[8] = RxBuffer.Data[8];
        Info.Phases = RxBuffer.Data[9];
        Info.RatedPower[0] = RxBuffer.Data[10];
        Info.RatedPower[1] = RxBuffer.Data[11];
        Info.RatedPower[2] = RxBuffer.Data[12];
        Info.RatedPower[3] = RxBuffer.Data[13];
        Info.RatedPower[4] = RxBuffer.Data[14];
        Info.RatedPower[5] = RxBuffer.Data[15];
        Info.FirmwareVer[0] = RxBuffer.Data[16];
        Info.FirmwareVer[1] = RxBuffer.Data[17];
        Info.FirmwareVer[2] = RxBuffer.Data[18];
        Info.FirmwareVer[3] = RxBuffer.Data[19];
        Info.FirmwareVer[4] = RxBuffer.Data[20];
        Info.ModuleName[0] = RxBuffer.Data[21];
        Info.ModuleName[1] = RxBuffer.Data[22];
        Info.ModuleName[2] = RxBuffer.Data[23];
        Info.ModuleName[3] = RxBuffer.Data[24];
        Info.ModuleName[4] = RxBuffer.Data[25];
        Info.ModuleName[5] = RxBuffer.Data[26];
        Info.ModuleName[6] = RxBuffer.Data[27];
        Info.ModuleName[7] = RxBuffer.Data[28];
        Info.ModuleName[8] = RxBuffer.Data[29];
        Info.ModuleName[9] = RxBuffer.Data[30];
        Info.ModuleName[10] = RxBuffer.Data[31];
        Info.ModuleName[11] = RxBuffer.Data[32];
        Info.ModuleName[12] = RxBuffer.Data[33];
        Info.ModuleName[13] = RxBuffer.Data[34];
        Info.FactoryName[0] = RxBuffer.Data[35];
        Info.FactoryName[1] = RxBuffer.Data[36];
        Info.FactoryName[2] = RxBuffer.Data[37];
        Info.FactoryName[3] = RxBuffer.Data[38];
        Info.FactoryName[4] = RxBuffer.Data[39];
        Info.FactoryName[5] = RxBuffer.Data[40];
        Info.FactoryName[6] = RxBuffer.Data[41];
        Info.FactoryName[7] = RxBuffer.Data[42];
        Info.FactoryName[8] = RxBuffer.Data[43];
        Info.FactoryName[9] = RxBuffer.Data[44];
        Info.FactoryName[10] = RxBuffer.Data[45];
        Info.FactoryName[11] = RxBuffer.Data[46];
        Info.FactoryName[12] = RxBuffer.Data[47];
        Info.FactoryName[13] = RxBuffer.Data[48];
        Info.SerialNumber[0] = RxBuffer.Data[49];
        Info.SerialNumber[1] = RxBuffer.Data[50];
        Info.SerialNumber[2] = RxBuffer.Data[51];
        Info.SerialNumber[3] = RxBuffer.Data[52];
        Info.SerialNumber[4] = RxBuffer.Data[53];
        Info.SerialNumber[5] = RxBuffer.Data[54];
        Info.SerialNumber[6] = RxBuffer.Data[55];
        Info.SerialNumber[7] = RxBuffer.Data[56];
        Info.SerialNumber[8] = RxBuffer.Data[57];
        Info.SerialNumber[9] = RxBuffer.Data[58];
        Info.SerialNumber[10] = RxBuffer.Data[59];
        Info.SerialNumber[11] = RxBuffer.Data[60];
        Info.SerialNumber[12] = RxBuffer.Data[61];
        Info.SerialNumber[13] = RxBuffer.Data[62];
        Info.RatedBusVoltage[0] = RxBuffer.Data[63];
        Info.RatedBusVoltage[1] = RxBuffer.Data[64];
        Info.RatedBusVoltage[2] = RxBuffer.Data[65];
        Info.RatedBusVoltage[3] = RxBuffer.Data[66];

        Data.Active = true;
        ReturnValue = true;
      }
      break;
      
    case SOLAX_CTLCODE_WRITE: // Control Code = 0x12 'write'
      break;
      
    case SOLAX_CTLCODE_EXECUTE: // Control Code = 0x13 'execute'
      break;
      
    default:
      //Debug("[ProcessRxMsg] unknown ControlCode");
      break;
  }
  
  return(ReturnValue);
}


void Solax::RequestNormalData(uint8_t Address) {
  static SolaxMessageT TxMsg;

  TxMsg.Source[0] = SOLAX_GATEWAY_ADDRESS;
  TxMsg.Source[1] = 0x00;
  TxMsg.Destination[0] = 0x00;
  TxMsg.Destination[1] = Address;
  TxMsg.ControlCode = 0x11;
  TxMsg.FunctionCode = 0x02;
  TxMsg.DataLength = 0x00;

  SendTxMsg(&TxMsg);
}

void Solax::RequestIdData(uint8_t Address) {
  static SolaxMessageT TxMsg;

  TxMsg.Source[0] = SOLAX_GATEWAY_ADDRESS;
  TxMsg.Source[1] = 0x00;
  TxMsg.Destination[0] = 0x00;
  TxMsg.Destination[1] = Address;
  TxMsg.ControlCode = 0x11;
  TxMsg.FunctionCode = 0x03;
  TxMsg.DataLength = 0x00;

  SendTxMsg(&TxMsg);
}

void Solax::RegisterAddress(uint8_t SerialNumber[14], uint8_t Address) {
  static SolaxMessageT TxMsg;

  TxMsg.Source[0] = 0x00;
  TxMsg.Source[1] = 0x00;
  TxMsg.Destination[0] = 0x01; // was 0x00, should be 0x01
  TxMsg.Destination[1] = 0x00;
  TxMsg.ControlCode = 0x10;
  TxMsg.FunctionCode = 0x01;
  TxMsg.DataLength = 0x0F;
  memcpy(TxMsg.Data, SerialNumber, 14);
  TxMsg.Data[14] = Address;

  SendTxMsg(&TxMsg);
}

void Solax::DiscoverDevice(void) { // offline_query 
  static SolaxMessageT TxMsg;

  TxMsg.Source[0] = 0x01;
  TxMsg.Source[1] = 0x00;
  TxMsg.Destination[0] = 0x00;
  TxMsg.Destination[1] = 0x00;
  TxMsg.ControlCode = 0x10;
  TxMsg.FunctionCode = 0x00;
  TxMsg.DataLength = 0x00;

  SendTxMsg(&TxMsg);
}

void Solax::SendDebugMsg(uint8_t DebugData[64]) {
  static SolaxMessageT TxMsg;

  TxMsg.Source[0] = 0xde;
  TxMsg.Source[1] = 0xad;
  TxMsg.Destination[0] = 0xaf;
  TxMsg.Destination[1] = 0xfe;
  TxMsg.ControlCode = 0x00;
  TxMsg.FunctionCode = 0x00;
  TxMsg.DataLength = 64;
  memcpy(TxMsg.Data, DebugData, 64);
  
  SendTxMsg(&TxMsg);
}

void Solax::SendTxMsg(SolaxMessageT *Buffer) {
  uint8_t SolaxTxByteCnt;
  uint16_t Checksum = 0;
  
  Buffer->Header[0] = 0xAA;
  Buffer->Header[1] = 0x55;

  SolaxTxByteCnt = 9 + Buffer->DataLength;
  
  Checksum = CalcChecksum((const SolaxMessageT) *Buffer);
  Buffer->Checksum[1] = Checksum >> 8;
  Buffer->Checksum[0] = Checksum;

  digitalWrite(TxEnablePin, HIGH);

  //Debug("Serial.write(" + String(SolaxTxByteCnt+2) + ")");
  Serial.write((const uint8_t *) Buffer, SolaxTxByteCnt);
  Serial.write(Buffer->Checksum[1]);
  Serial.write(Buffer->Checksum[0]);
  Serial.flush();

  digitalWrite(TxEnablePin, LOW);
}


// Function: CalcChecksum()
//  Calculates the checksum of all bytes in a buffer.
//  REF{1}, table in chapter 4: Checksum = Header + Source Address + Destination Address + Control Code + Function Code + Data Length + Data0 + .. + Data (N-1)
// Parameter:
//  struct SolaxMessageT *: Provides the header, address, function and data bytes as input for checksum calculation.
// Return:
//  uint16_t: Calculated checksum
uint16_t Solax::CalcChecksum(const struct SolaxMessageT Buffer) {
  uint8_t i;
  uint16_t Checksum = 0;

  Checksum += Buffer.Header[0];
  Checksum += Buffer.Header[1];
  Checksum += Buffer.Source[0];
  Checksum += Buffer.Source[1];
  Checksum += Buffer.Destination[0];
  Checksum += Buffer.Destination[1];
  Checksum += Buffer.ControlCode;
  Checksum += Buffer.FunctionCode;
  Checksum += Buffer.DataLength;

  for (i = 0; i <= (Buffer.DataLength); i++) {
    Checksum += Buffer.Data[i];
  }
/*  
  //Debug("[CRC] Header[0]: " + String(Buffer.Header[0], HEX));
  //Debug("[CRC] Header[1]: " + String(Buffer.Header[1], HEX));
  //Debug("[CRC] Source[0]: " + String(Buffer.Source[0], HEX));
  //Debug("[CRC] Source[1]: " + String(Buffer.Source[1], HEX));
  //Debug("[CRC] Destination[0]: " + String(Buffer.Destination[0], HEX));
  //Debug("[CRC] Destination[1]: " + String(Buffer.Destination[1], HEX));
  //Debug("[CRC] ControlCode: " + String(Buffer.ControlCode, HEX));
  //Debug("[CRC] FunctionCode: " + String(Buffer.FunctionCode, HEX));
  //Debug("[CRC] DataLength: " + String(Buffer.DataLength, HEX));
  //Debug("[CRC] New Checksum: " + String(Checksum, HEX));
*/  
  return(Checksum);
}

//  Calculates the average power every 60 measurement values.
// Parameter:
//  uint16_t Actual power
// Return:
//  uint16_t: Calculated average power
uint16_t Solax::CalcAvgPower(uint16_t ActPower) {
    static uint16_t AvgPowerBuffer[15];
    static uint8_t idx = 0;
    static uint8_t Stat_Cnt = 0;
    static uint16_t AvgPower = 0;
    uint32_t temp = 0;
    
    AvgPowerBuffer[idx] = ActPower;
    
    if(idx >= 14) {
      idx = 0;
      for(int i = 0; i<15; i++) {
        temp += AvgPowerBuffer[i];
      }
      AvgPower = temp / 15;
      
      Stat_AvgPower[Stat_Idx] += AvgPower;
      Stat_Cnt++;
      if(Stat_Cnt >= 10) {
        Stat_Cnt = 0;
        Stat_AvgPower[Stat_Idx] = Stat_AvgPower[Stat_Idx] / 10;
        Stat_PwrToday[Stat_Idx] = Data.YieldToday;
        Stat_Idx++;
      }
    }
    else idx++;

    return(AvgPower);
}
