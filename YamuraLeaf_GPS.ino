/*
   Yamura Leaf - bluetooth GPS for Dual XGPS160
   BBS 3/2021
   send GPS position info to hub

   for ESP32 with built in bluetooth module
*/
#define DEBUG_PRINT
#define ESP8266_LED 5

#define GPRMC 0
#define GPGGA 1

#ifdef ESP32
  #include <esp_now.h>
  #include <WiFi.h>
#endif
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#endif
#include "BluetoothSerial.h"

// data package structure
#define MESSAGE_LEN 50
struct LeafData
{
  char leafType;            //  1 byte  - type, in this case 'G' for GPS
  unsigned long timeStamp;  //  4 bytes - micros() value of sample
  char nmeaTime[15];        // 10 bytes of nmea time string in form hhmmss.sss
  char gpsLatitude[15];       //  9 bytes of nmea latitude in form ddmm.mmmm              
  char gpsLongitude[15];      // 10 bytes of nmea longitude in form dddmm.mmmm              
};
// data package union to do conversion to bytes
union DataToSend
{
  struct LeafData leafData;
  uint8_t dataBytes[MESSAGE_LEN];
} toSend;

char gpsRecieved[128];

unsigned long disconnectRetryTime = 10000000;
unsigned long lastTime;
unsigned long curTime;
unsigned long targetInterval = 25000;  // (sample at 40Hz)
unsigned long sampleInterval = 25000;
unsigned long lastInterval = 0;
uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0xF6, 0x45, 0x80};


BluetoothSerial SerialBT;
String btName = "XGPS160-45E134";
char *btPin = "1234"; //<- standard pin would be provided by default
bool btConnected;
//
//
//
void setup()
{
  Serial.begin(115200);

  int blinkState = HIGH;
  pinMode(ESP8266_LED, OUTPUT); // built in LED
  
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  // This is the mac address of this device
  //#ifdef DEBUG_PRINT
  Serial.println();
  Serial.print("YamuraLeaf-GPS at ");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  //#endif
  // Init ESPNow with a fallback logic
  Serial.println("Begin ESP-NOW initialization");
  InitESPNow();
  //esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_peer_info *peer = new esp_now_peer_info();
  
  peer->peer_addr[0]= hub_addr[0];
  peer->peer_addr[1]= hub_addr[1];
  peer->peer_addr[2]= hub_addr[2];
  peer->peer_addr[3]= hub_addr[3];
  peer->peer_addr[4]= hub_addr[4];
  peer->peer_addr[5]= hub_addr[5];
 //peer.lmk = NULL;
  peer->channel = 1;
  peer->encrypt = false;
  peer->priv = NULL;
  
  esp_now_add_peer((const esp_now_peer_info_t*)peer);// hub_addr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_register_send_cb(OnDataSent);

  // set last and current sample 
  lastTime = micros();
  curTime = micros();
  // leaf type
  toSend.leafData.leafType = 'G';

  for(int cnt = 0; cnt < 30; cnt++)
  {
      digitalWrite(ESP8266_LED, blinkState); // built in LED
      delay(100);
      blinkState = blinkState == LOW ? blinkState == HIGH :blinkState == LOW; 
  }

  Serial.println("Begin bluetooth initialization");
  // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
  // to resolve name to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices bluetooth address and device names
  Serial.print("Attempt to connect to "); Serial.println(btName);

  SerialBT.begin("ESP32test", true); 
  ConnectToGPS();
  digitalWrite(ESP8266_LED, LOW); // built in LED
  Serial.println();
  curTime = micros();
  lastTime = curTime;
}
//
// read GPS, push data to hub
// in this case, we're waiting for data from GPS unit, so just send it when it's ready
//
void loop() 
{
  curTime = micros();
  if(SerialBT.available() > 0)
  {
    // got a message of some sort
    lastTime = curTime;
    String outStr = "";
    byte inByte;
    int charIdx = 0;
    // read available byte(s)
    while (SerialBT.available()) 
    {
      inByte = SerialBT.read();
      if(inByte == 10)
      {
        break;
      }
      gpsRecieved[charIdx]=(char)inByte;
      charIdx++;
    }
    // here are the interesting messages
    // $GPRMC - 11 fields
    //                     A=valid    latitude                longitude
    //         hhmmss.sss  V=invalid  ddmm.mmmm               dddmm.mmmm      Speed Course  ddmmyy M  CS
    // $GPRMC, 161229.487, A,         3723.2475,  N,          12158.3416, W,  0.13, 309.62, 120598, , *10
    // token 1 = type (GPRMC)
    // token 2 = nmea time
    // token 4 = latitude
    // token 6 = longitude
    char *token = NULL;
    char buf[256];
    // get the first token - message type. accept only GPRMC messages
    token = strtok(gpsRecieved,",");
    int tokenCount = 0;
    if(strcmp(token,"$GPRMC") != 0)
    {
      return;
    }
    toSend.leafData.timeStamp = micros();
    while(token != NULL)
    {
      tokenCount++;
      switch(tokenCount)
      {
        case 2:   // nmea time
          // must be 10 characters in format hhmmss.sss
          if(strlen(token) != 10)
          {
            return;
          }
          strcpy(toSend.leafData.nmeaTime, token);
          break;
        case 4:   // nmea latitude in format ddmm.mmmmm
          if(strlen(token) != 10)
          {
            return;
          }
          strcpy(toSend.leafData.gpsLatitude, token);
          break;
        case 6:   // nmea longitude in format dddmm.mmmmm
          if(strlen(token) != 11)
          {
            return;
          }
          strcpy(toSend.leafData.gpsLongitude, token);
          break;
        default:  // anll others
          break;
      }
      token = strtok(NULL,",");
    }
    // bad data packet
    if((tokenCount != 11) ||
       (strcmp(toSend.leafData.nmeaTime, "000000.000") == 0))
    {
      return;
    }
    sendData();
  }
  // is connection still alive? if not retry
  else
  {
    // 30 seconds since last message recieved, retry
    if(curTime - lastTime > disconnectRetryTime)
    {
      #ifdef DEBUG_PRINT
      Serial.print(disconnectRetryTime/1000000); Serial.println(" seconds since last message or reconnect attempt, reattempt GPS connection");
      #endif
      ConnectToGPS();
      lastTime = curTime;
    }
  }
}
//
// send data
//
void sendData()
{
  #ifdef DEBUG_PRINT
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.print(toSend.leafData.leafType);
  Serial.print(" Time ");
  Serial.print(toSend.leafData.timeStamp);
  Serial.print(" Values nmeaTime ");
  Serial.print(toSend.leafData.nmeaTime); Serial.print(" lat ");
  Serial.print(toSend.leafData.gpsLatitude); Serial.print(" long ");
  Serial.print("\n");
  #endif
  uint8_t result = esp_now_send(hub_addr, &toSend.dataBytes[0], sizeof(LeafData));
  #ifdef DEBUG_PRINT
  switch(result)
  {
    case 0:
      break;
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("\nESPNOW not initialized Error");
      break;
    case ESP_ERR_ESPNOW_ARG:
      Serial.println("\nInvalid Argument Error");
      break;
    case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("\nOut of memory Error");
      break;
    case ESP_ERR_ESPNOW_FULL:
      Serial.println("\nPeer list full Error");
      break;
    case ESP_ERR_ESPNOW_NOT_FOUND:
      Serial.println("\nPeer not found Error");
      break;
    case ESP_ERR_ESPNOW_INTERNAL:
      Serial.println("\nInternal Error");
      break;
    case ESP_ERR_ESPNOW_EXIST:
      Serial.println("\nPeer has existed Error");
      break;
    case ESP_ERR_ESPNOW_IF:
      Serial.println("\nInterface error Error");
      break;
    default:
      Serial.print("Other message send error ");
      Serial.print(result);
      Serial.print(" error base ");
      Serial.println(ESP_ERR_ESPNOW_BASE);
      break;
  }
  #endif
}
//
// Init ESP Now with fallback
//
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == 0) 
  {
    //#ifdef DEBUG_PRINT
    Serial.println("ESPNow Init Success");
    //#endif
  }
  else {
    //#ifdef DEBUG_PRINT
    Serial.println("ESPNow Init Failed");
    //#endif
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}
//
//
//
void ConnectToGPS()
{
  btConnected = SerialBT.connect(btName);
  
  if(btConnected) 
  {
    Serial.println("Connected Succesfully!");
  }
  else 
  {
    while(!SerialBT.connected(10000)) 
    {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) 
  {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
}
//
// callback when data is sent from Master to Slave
//
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  if(status != 0)
  {
    char macStr[18];
    #ifdef DEBUG_PRINT
    Serial.print("Last Packet Sent to: ");
    for(int idx = 0; idx < 6; idx++)
    {
      Serial.print(mac_addr[idx], HEX); Serial.print(" ");
    }
    Serial.print(" Failed: ");
    Serial.println(status);
    #endif
  }
}
