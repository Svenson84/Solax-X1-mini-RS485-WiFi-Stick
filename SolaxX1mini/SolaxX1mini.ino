// This software provides a local webinterface via a wifi connection for the "Solax X1 Mini" power inverter.
// An ESP8266 (Wemos D1 mini) with an external RS485 transceiver (MAX 3485) is used as a gateway between the RS485 of the Solax power inverter and your private LAN/WiFi. 
// It does only use the RS485 to communicate with the Solax power inverter!
// It does NOT required the official Solax WIFI or LAN stick!
// It does NOT use the Solax cloud connection or Solax API! 
// It does NOT require any internet connection!
// 
// This code uses, and/or is partly based on, and/or is inspired by the following projects:
// https://github.com/JensJordan/solaXd
// https://github.com/syssi/esphome-modbus-solax-x1
// https://github.com/chartjs/Chart.js
//
// Using following external data (images, fonts, etc.)
// HTML Launch Picture: https://www.pexels.com/de-de/foto/innovation-erneuerbare-energie-umweltfreundlich-solarplatten-6729421/
// Font: https://fonts.google.com/specimen/Lato#standard-styles
// Icons: https://github.com/Remix-Design/RemixIcon
// FavIcon: https://www.flaticon.com/de/kostenlose-icons/solar
//
// Disclaimer:
// Feel free to use this code on your own risk ;)
//
// Referenced documents:
// REF{1} https://github.com/syssi/esphome-modbus-solax-x1/blob/main/docs/SolaxPower_Single_Phase_External_Communication_Protocol_-_X1_Series_V1.2.pdf
// REF{2} https://www.solaxpower.com/wp-content/uploads/2017/01/X1-Mini-Install-Manual.pdf

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>

#include "Solax.h"

#define LED_PIN                       2     // Wemos D1 (D4) = GPIO2 --> On-board LED
#define WIFI_RESET_PIN                13    // Wemos D1 (D7) = GPIO13 --> internal Pull-Up. Add Jumper to GND for Wifi Reset
#define SOLAX_RS485_TX_ENABLE_PIN     5     // Wemos D1 (D1) = GPIO5 --> Transmitter active HIGH, Receiver active LOW
#define HTTP_TIMEOUT_TIME             3     // sec
#define INFLUX_UPDATE_INTERVAL        60    // sec

#define WIFI_HOSTNAME                 "SolaxWifi"
#define WIFI_CONFIG_SSID              "SolaxWifi-Setup"
#define WIFI_CONFIG_PWD               "Solax"
#define OTA_PWD                       "Solax"
#define INFLUX_URI                    "http://192.168.0.123:8086/write?db=MyDatabase"
#define INFLUX_USER                   "username"
#define INFLUX_PASS                   "password"
#define INFLUX_CLIENT_ID              "ESP8266"
#define INFLUX_TABLE                  "Solax"

Solax Inverter(SOLAX_RS485_TX_ENABLE_PIN);
WiFiManager wifiManager;
ESP8266WebServer WebServer(80);
WiFiClient WifiClient;

bool wifiReset = false;
bool espReboot = false;
uint8_t retryCnt = 0;

uint32_t Now = 0;
uint32_t uptime_seconds = 0;

int32_t WiFiRSSI = 0;
int32_t WiFiReconnects = 0;


void UpdateInfluxDB() {
  static uint32_t LastTime = 0;

  Now = millis();
  if(Inverter.Data.Active == true && (Now - LastTime) > (INFLUX_UPDATE_INTERVAL * 1000U)) {
    LastTime = Now;
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
   
      http.begin(WifiClient, INFLUX_URI);
      http.setAuthorization(INFLUX_USER, INFLUX_PASS);
      http.addHeader("User-Agent", INFLUX_CLIENT_ID);
      http.addHeader("Connection", "close");
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");

      String msg;
      msg += String(INFLUX_TABLE) + ",source=" + String(INFLUX_CLIENT_ID) + " ";
      msg += "RSSI="+String(WiFiRSSI);
      msg += ",Temperature="+String(Inverter.Data.Temperature);
      msg += ",PV1Voltage="+String(Inverter.Data.PV1Voltage/10.0);
      msg += ",PV1Current="+String(Inverter.Data.PV1Current/10.0);
      msg += ",CurrentOutput="+String(Inverter.Data.CurrentOutput/10.0);
      //msg += ",GridVoltage="+String(Inverter.Data.GridVoltage);
      //msg += ",GridFrequency="+String(Inverter.Data.GridFrequency);
      //msg += ",PowerOutput="+String(Inverter.Data.PowerOutput);
      msg += ",PowerOutput="+String(Inverter.Data.AvgPowerOutput);
      msg += ",InverterMode="+String(Inverter.Data.InverterMode);
      msg += ",YieldToday="+String(Inverter.Data.YieldToday/10.0);
      msg += ",YieldTotal="+String(Inverter.Data.YieldTotal/10.0);


      int httpCode = http.POST(msg);
      if (httpCode == 204) {
        // [UpdateInfluxDB] Update successful
        // HTTP Status 204: Success, no Content
        // nothing to do
        retryCnt = 0;
      }
      else {
        // [UpdateInfluxDB] Update failed
        retryCnt++;
        if(retryCnt >= 3) espReboot = true;
      }

      http.end();
    }
  }
}

void status_led(uint8_t on) {
  if(on) {
    digitalWrite(LED_PIN, LOW);
  }
  else {
    digitalWrite(LED_PIN, HIGH);
  }
}

void UptimeUpdate() {
  static uint16_t delta_millis = 0;
  static uint32_t last_update = 0;

  Now = millis();
  delta_millis += (Now - last_update);
  last_update = Now;
  
  while(delta_millis >= 1000) {
    uptime_seconds++;
    delta_millis -= 1000;
  }
}

void CheckWifiStatus() {
  bool res;
  if (WiFi.status() != WL_CONNECTED) {
    // WiFi connection lost... try to reconnect!
    res = wifiManager.autoConnect(WIFI_CONFIG_SSID, WIFI_CONFIG_PWD);
    if(!res) {
      status_led(false);
    } 
    else {
      status_led(true);
      WiFiReconnects++;
    }
  }
  else {
	  WiFiRSSI = WiFi.RSSI();
  }
}

void SendJsonIdInfo() {
  String json = "{\n";
  json += "  \"NotUsed0\": " + String(Inverter.Info.NotUsed[0]) + ",\n";
  json += "  \"NotUsed1\": " + String(Inverter.Info.NotUsed[1]) + ",\n";
  json += "  \"NotUsed2\": " + String(Inverter.Info.NotUsed[2]) + ",\n";
  json += "  \"NotUsed3\": " + String(Inverter.Info.NotUsed[3]) + ",\n";
  json += "  \"NotUsed4\": " + String(Inverter.Info.NotUsed[4]) + ",\n";
  json += "  \"NotUsed5\": " + String(Inverter.Info.NotUsed[5]) + ",\n";
  json += "  \"NotUsed6\": " + String(Inverter.Info.NotUsed[6]) + ",\n";
  json += "  \"NotUsed7\": " + String(Inverter.Info.NotUsed[7]) + ",\n";
  json += "  \"NotUsed8\": " + String(Inverter.Info.NotUsed[8]) + ",\n";
  json += "  \"Phases\": " + String(Inverter.Info.Phases) + ",\n";
  json += "  \"RatedPower\": \"" + String(Inverter.Info.RatedPower) + "\",\n";
  json += "  \"FirmwareVer\": \"" + String(Inverter.Info.FirmwareVer) + "\",\n";
  json += "  \"ModuleName\": \"" + String(Inverter.Info.ModuleName) + "\",\n";
  json += "  \"FactoryName\": \"" + String(Inverter.Info.FactoryName) + "\",\n";
  json += "  \"SerialNumber\": \"" + String(Inverter.Info.SerialNumber) + "\",\n";
  json += "  \"RatedBusVoltage\": \"" + String(Inverter.Info.RatedBusVoltage) + "\n";
  json += "}";
  
  WebServer.send(200, "application/json", json);
}


void SendJsonStatus() {
  String json = "{\n";

  json += "  \"mode_name\": \"";
  switch(Inverter.Data.InverterMode) { 
    case 0: json += "Waiting"; break;
    case 1: json += "Checking"; break;
    case 2: json += "On-Grid"; break;
    case 3: json += "Fault"; break;
    default: json += "Unkown"; break;
  }
  json += "\",\n";

  json += "  \"mode\": " + String(Inverter.Data.InverterMode) + ",\n";
  json += "  \"temperature\": " + String(Inverter.Data.Temperature) + ",\n";

  json += "  \"error_bits\": \"";
  if(Inverter.Data.ErrorMessage == 0) json += "None";
  else {
    // Error Byte 0
    if((Inverter.Data.ErrorMessage & (1 << 0)) == true) json += "TzProtectFault ";
    if((Inverter.Data.ErrorMessage & (1 << 1)) == true) json += "MainsLostFault ";
    if((Inverter.Data.ErrorMessage & (1 << 2)) == true) json += "GridVoltFault ";
    if((Inverter.Data.ErrorMessage & (1 << 3)) == true) json += "GridFreqFault ";
    if((Inverter.Data.ErrorMessage & (1 << 4)) == true) json += "PLLLostFault ";
    if((Inverter.Data.ErrorMessage & (1 << 5)) == true) json += "BusVoltFault ";
    if((Inverter.Data.ErrorMessage & (1 << 6)) == true) json += "BIT06 ";
    if((Inverter.Data.ErrorMessage & (1 << 7)) == true) json += "OciFault ";
    // Error Byte 1
    if((Inverter.Data.ErrorMessage & (1 << 8)) == true) json += "DciOCPFault ";
    if((Inverter.Data.ErrorMessage & (1 << 9)) == true) json += "ResidualCurrentFault ";
    if((Inverter.Data.ErrorMessage & (1 << 10)) == true) json += "PvVoltFault ";
    if((Inverter.Data.ErrorMessage & (1 << 11)) == true) json += "Ac10MinsVoltageFault ";
    if((Inverter.Data.ErrorMessage & (1 << 12)) == true) json += "IsolationFault ";
    if((Inverter.Data.ErrorMessage & (1 << 13)) == true) json += "TemperatureOverFault ";
    if((Inverter.Data.ErrorMessage & (1 << 14)) == true) json += "FanFault ";
    if((Inverter.Data.ErrorMessage & (1 << 15)) == true) json += "BIT15 ";
    // Error Byte 2
    if((Inverter.Data.ErrorMessage & (1 << 16)) == true) json += "SpiCommsFault ";
    if((Inverter.Data.ErrorMessage & (1 << 17)) == true) json += "SciCommsFault ";
    if((Inverter.Data.ErrorMessage & (1 << 18)) == true) json += "BIT18 ";
    if((Inverter.Data.ErrorMessage & (1 << 19)) == true) json += "InputConfigFault ";
    if((Inverter.Data.ErrorMessage & (1 << 20)) == true) json += "EepromFault ";
    if((Inverter.Data.ErrorMessage & (1 << 21)) == true) json += "RelayFault ";
    if((Inverter.Data.ErrorMessage & (1 << 22)) == true) json += "SampleConsistenceFault ";
    if((Inverter.Data.ErrorMessage & (1 << 23)) == true) json += "ResidualCurrentDeviceFault ";
    // Error Byte 3
    if((Inverter.Data.ErrorMessage & (1 << 24)) == true) json += "BIT24 ";
    if((Inverter.Data.ErrorMessage & (1 << 25)) == true) json += "BIT25 ";
    if((Inverter.Data.ErrorMessage & (1 << 26)) == true) json += "BIT26 ";
    if((Inverter.Data.ErrorMessage & (1 << 27)) == true) json += "BIT27 ";
    if((Inverter.Data.ErrorMessage & (1 << 28)) == true) json += "BIT28 ";
    if((Inverter.Data.ErrorMessage & (1 << 29)) == true) json += "DCIDeviceFault ";
    if((Inverter.Data.ErrorMessage & (1 << 30)) == true) json += "OtherDeviceFault ";
    if((Inverter.Data.ErrorMessage & (1 << 31)) == true) json += "BIT31 ";
  }
  json += "\",\n";
  
  json += "  \"dc_voltage\": " + String(Inverter.Data.PV1Voltage / 10.0) + ",\n";
  json += "  \"dc_current\": " + String(Inverter.Data.PV1Current / 10.0) + ",\n";
  json += "  \"ac_voltage\": " + String(Inverter.Data.GridVoltage / 10.0) + ",\n";
  json += "  \"ac_current\": " + String(Inverter.Data.CurrentOutput / 10.0) + ",\n";
  json += "  \"ac_frequency\": " + String(Inverter.Data.GridFrequency / 100.0) + ",\n";
  json += "  \"ac_power\": " + String(Inverter.Data.PowerOutput) + ",\n";
  json += "  \"avg_ac_power\": " + String(Inverter.Data.AvgPowerOutput) + ",\n";
  json += "  \"max_ac_power\": " + String(Inverter.Data.MaxPowerOutput) + ",\n";
  json += "  \"stat_ac_power\": [";
  if(Inverter.Stat_Idx > 0) {
    for (int i=0; i<(Inverter.Stat_Idx-1); i++) {
      json += String(Inverter.Stat_AvgPower[i]) + ", ";
    }
    json += String(Inverter.Stat_AvgPower[Inverter.Stat_Idx-1]);
  }
  json += "],\n";
  json += "  \"energy_today\": " + String(Inverter.Data.YieldToday / 10.0) + ",\n";
  json += "  \"stat_energy_today\": [";
  if(Inverter.Stat_Idx > 0) {
    for (int i=0; i<(Inverter.Stat_Idx-1); i++) {
      json += String(Inverter.Stat_PwrToday[i]) + ", ";
    }
    json += String(Inverter.Stat_PwrToday[Inverter.Stat_Idx-1]);
  }
  json += "],\n";
  json += "  \"energy_total\": " + String(Inverter.Data.YieldTotal / 10.0) + ",\n";
  json += "  \"runtime\": " + String(uptime_seconds) + ",\n";
  json += "  \"runtime_total\": " + String(Inverter.Data.RuntimeTotal) + ",\n";
  json += "  \"wifi_ssid\": \"" + String(WiFi.SSID()) + "\",\n";
  json += "  \"wifi_ip\": \"" + WiFi.localIP().toString() + "\",\n";
  json += "  \"wifi_rssi\": " + String(WiFiRSSI) + ",\n";
  json += "  \"wifi_reconnects\": " + String(WiFiReconnects) + "\n";
  json += "}";

  WebServer.send(200, "application/json", json);
}

void handleRoot(){
  WebServer.sendHeader("Location", "/index.html", true);
  WebServer.send(302, "text/plain","");
}
 
void handleWebRequests(){
  String file;
  String type;
  String uri = WebServer.uri();

  if(uri.startsWith("/update.json")) {
    SendJsonStatus();
  }
  else if(uri.startsWith("/IdInfo.json")) {
    SendJsonIdInfo();
  }
  else {
    if(uri.startsWith("/index.html")) {
      file = "index.html";
      type = "text/html";
    }
    else if(uri.startsWith("/solax.css")) {
      file = "solax.css";
      type = "text/css";
    }
    else if(uri.startsWith("/remixicon.css")) {
      file = "remixicon.css";
      type = "text/css";
    }
    else if(uri.startsWith("/remixicon.eot")) {
      file = "remixicon.eot";
      type = "font/eot";
    }
    else if(uri.startsWith("/remixicon.ttf")) {
      file = "remixicon.ttf";
      type = "font/ttf";
    }
    else if(uri.startsWith("/remixicon.woff")) {
      file = "remixicon.woff";
      type = "font/woff";
    }
    else if(uri.startsWith("/remixicon.woff2")) {
      file = "remixicon.woff2";
      type = "font/woff2";
    }
    else if(uri.startsWith("/Lato-Regular.ttf")) {
      file = "Lato-Regular.ttf";
      type = "font/ttf";
    }
    else if(uri.startsWith("/Lato-Bold.ttf")) {
      file = "Lato-Bold.ttf";
      type = "font/ttf";
    }
    else if(uri.startsWith("/solax.js")) {
      file = "solax.js";
      type = "application/javascript";
    }
    else if(uri.startsWith("/chart.min.js")) {
      file = "chart.min.js";
      type = "application/javascript";
    }
    else if(uri.startsWith("/favicon.ico")) {
      file = "favicon.ico";
      type = "image/x-icon";
    }
    else if(uri.startsWith("/touch-icon-iphone.png")) {
      file = "touch-icon-iphone.png";
      type = "image/png";
    }
    
    if(file != "") {
      File dataFile = LittleFS.open(file, "r");
      if (WebServer.hasArg("download")) type = "application/octet-stream";
      if(dataFile) {
        WebServer.streamFile(dataFile, type);
      }
      dataFile.close();
    }
    else {
      String message = "404 - invalid request!\n";
      WebServer.send(404, "text/plain", message);
    }
  }
}


void setup() { 
  Inverter.begin();  

  // Configure GPIO pin for wifi reset input
  pinMode(WIFI_RESET_PIN, INPUT_PULLUP);

  // Configure GPIO pin for status LED
  pinMode(LED_PIN, OUTPUT);
  status_led(false);

  // Check if reset of wifi data is requested by pulling WIFI_RESET_PIN to GND
  if(digitalRead(WIFI_RESET_PIN) == 0) {
    wifiManager.resetSettings();
    while(1) {  
      delay(100);
      status_led(true);
      delay(100);
      status_led(false);
    }
  }

  //WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  wifiManager.setDebugOutput(false);
  wifiManager.setHostname(WIFI_HOSTNAME);
	wifiManager.setEnableConfigPortal(true);
  wifiManager.setWiFiAutoReconnect(false);
	wifiManager.setCleanConnect(true);
	wifiManager.setConnectRetries(10);
	wifiManager.setConnectTimeout(6);

  bool res;
  //res = wifiManager.autoConnect(WIFI_CONFIG_SSID, WIFI_CONFIG_PWD);
  res = wifiManager.autoConnect(WIFI_CONFIG_SSID);
  if(!res) {
    // Failed to connect... reboot!
    ESP.restart();
  }
  
  // WiFi connected!
  status_led(true);
  
  //ArduinoOTA.setPassword(OTA_PWD);
  ArduinoOTA.setHostname(WIFI_HOSTNAME);
  ArduinoOTA.begin();
  
  LittleFS.begin();
  //LittleFS.format();

  WebServer.on("/", handleRoot);
  WebServer.onNotFound(handleWebRequests);
  WebServer.begin();
} 
 
void loop() {
  if(wifiReset) {
    wifiManager.resetSettings();
    wifiManager.reboot();
  }

  if(espReboot) {
    ESP.restart();
  }

  ArduinoOTA.handle();
  UptimeUpdate();
  CheckWifiStatus();
  Inverter.loop();
  UpdateInfluxDB();
  WebServer.handleClient();
}
