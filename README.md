# Solax-X1-mini-RS485-WiFi-Stick

RS485 WiFi Stick is a project to monitor the Solax X1 mini via RS485 interface.  
Optional data can be stored in a InfluxDB.

![Webinterface](/docs/Screenshot_Webinterface.jpeg "Webinterface")

## What it does and what it doesn't?
This software provides a local webinterface via a wifi connection for the "Solax X1 Mini" power inverter.  
An ESP8266 (Wemos D1 mini) with an external RS485 transceiver (MAX 3485) is used as a gateway between the RS485 of the Solax power inverter and your private LAN/WiFi.   
It does only use the RS485 to communicate with the Solax power inverter!  
It does NOT required the official Solax WIFI or LAN stick!  
It does NOT use the Solax cloud connection or Solax API!  
It does NOT require any internet connection!  

## References
This code uses, and/or is partly based on, and/or is inspired by the following projects:  
https://github.com/JensJordan/solaXd  
https://github.com/syssi/esphome-modbus-solax-x1

## Hardware

* ESP8266 (e.g. Wemos D1 mini)
* RS485-to-TTL module with MAX3485
* USB connector / cable
* RJ45 connector / cable
* Optional: Housing

### Connection of RS485-TTL module

```
               RS485                        UART
┌─────────┐              ┌─────────────┐           ┌─────────────────┐
│         │              │          RXD│<--------->│TX               │
│  Solax  │<---- A+ ---->│  RS485    DE│<--\       │       WEMOS D1  │
│ X1 Mini │<---- B- ---->│  to TTL  /RE│<---+----->│D1      ESP8266  │
│         │<--- GND ---->│  module  TXD│<--------->│RX               │
│         │              │             │           │                 │
│         │              │          VCC│<--------->│3.3V          VCC│<--
│         │              │          GND│<--------->│GND           GND│<--
└─────────┘              └─────────────┘           └─────────────────┘
```

### Modification of WEMOS D1 mini board
Serial resistors for TX and RX on the Wemos D1 mini have been removed.  
Remark: I am not sure if it is mandatory to remove thoses resistors. In the beginning i had troubles to get the RS485 working, so i removed thoses resistors and never tested again with populated resistors!  

Be careful: if you remove thoses resistors, a flashing/programming via USB interface of the WEMOS D1 mini is not possible anymore.  
You have to flash the software before removing of the resistors. With flashing the software, also a Over-The-Air  (OTA) will be flashed. Then it is still possible to flash an update via OTA if the resistors have been removed.

### Modification of RS485 board
I was not able to find a RS485 board, that was using the MAX3485 (3.3V version) instead of the regular MAX485 (5V version). 
It might work also with the MAX485, but i used only the MAX3485 - without any issues all the time. 

![RS485 Board](/docs/RS485_board_comment.jpeg "RS485 Board")

Originally the DE/RE pins are controlled automatically. By cutting the PCB track and connecting the DE/RE pins to the ESP8266 pin D1 by a single wire, the flow controll is handled by the ESP8266 SW.


### Solax X1 Mini RS485 interface

| Pin     | Purpose      | RS485-to-TTL pin  | Color T-568B |
| :-----: | :----------- | :---------------- | ------------ |
|    1    | RefGen       |                   |              |
|    2    | Com/DRM0     |                   |              |
|    3    | GND_COM      |                   |              |
|    4    | **A+**       | **A+**            | Blue         |
|    5    | **B-**       | **B-**            | Blue-White   |
|    6    | E_Stop       |                   |              |
|    7    | **GND_COM**  | **GND**           | Brown-White  |
|    8    | --           |                   |              |

## Software

The available code is available for the Arduino IDE and developed with a Wemos D1 mini ESP8266.

### Website
The application uses LittleFS. The Webinterface can also be uploaded directly with the Arduino IDE by a LittleFS file upload to the target hardware. 

### InfluxDB
ToDo

## Mechanical

### Housing
