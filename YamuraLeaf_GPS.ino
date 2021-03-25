/*
   Yamura Leaf - bluetooth GPS for Dual XGPS160
   BBS 3/2021
   send GPS position info to hub

   for ESP32 with built in bluetooth module
*/
//#define PRINT_DEBUG
#define TARGET_INTERVAL 50000
#define TIMESTAMP_REQUEST_INTERVAL 5000000
#define DISCONNECT_RETRY 10000000 // loss of BT for GPS unit


#include <esp_now.h>
#include <WiFi.h>
#include "BluetoothSerial.h"
#include "DataStructures.h"
// timestamp (H and T types)
TimeStampPacket timeStamp;
// digital/A2D data (I type)
GPSPacket gpsPacket;

char gpsRecieved[128];

unsigned long lastSampleTime;
unsigned long currentSampleTime;
unsigned long targetInterval = TARGET_INTERVAL;
unsigned long lastSampleInterval = 0;
unsigned long timestampAdjust = 0;
bool isLogging = false;
unsigned long sentCount = 0;
unsigned long errorCount = 0;
// hub MAC addres
uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0xF6, 0x45, 0x80};
esp_now_peer_info *hub;

BluetoothSerial SerialBT;
String btName = "XGPS160-45E134";
char *btPin = "1234"; //<- standard pin would be provided by default
bool btConnected;

#define ESP8266_LED 5
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
  Serial.println();
  Serial.print("YamuraLeaf GPS at ");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // register for receive callback for data send response
  esp_now_register_recv_cb(OnDataRecv);
  // send message callback
  esp_now_register_send_cb(OnDataSent);
  // add hub as a peer
  hub = new esp_now_peer_info();
  hub->peer_addr[0]= hub_addr[0];
  hub->peer_addr[1]= hub_addr[1];
  hub->peer_addr[2]= hub_addr[2];
  hub->peer_addr[3]= hub_addr[3];
  hub->peer_addr[4]= hub_addr[4];
  hub->peer_addr[5]= hub_addr[5];
  hub->channel = 1;
  hub->encrypt = false;
  hub->priv = NULL;
  esp_now_add_peer((const esp_now_peer_info_t*)hub);

  // set last and current sample 
  lastSampleTime = micros();
  currentSampleTime = micros();
  // leaf type
  gpsPacket.packet.leafType[0] = 'G';


  #ifdef PRINT_DEBUG
  Serial.println("Begin bluetooth initialization");
  // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
  // to resolve name to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices bluetooth address and device names
  Serial.print("Attempt to connect to "); Serial.println(btName);
  #endif
  SerialBT.begin("ESP32test", true); 
  ConnectToGPS();
  digitalWrite(ESP8266_LED, LOW); // built in LED
  #ifdef PRINT_DEBUG
  Serial.println();
  #endif
  for(int cnt = 0; cnt < 30; cnt++)
  {
      digitalWrite(ESP8266_LED, blinkState);
      delay(100);
      blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
  }
  currentSampleTime = micros();
  lastSampleTime = currentSampleTime;
  Serial.println("Running");
}
//
// read GPS, push data to hub
// in this case, we're waiting for data from GPS unit, so just send it when it's ready
//
void loop() 
{
  currentSampleTime = micros();
  if(isLogging)
  {
    if(SerialBT.available() > 0)
    {
      // got a message of some sort
      lastSampleTime = currentSampleTime;
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
      gpsPacket.packet.timeStamp = currentSampleTime - timestampAdjust;
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
            strcpy(gpsPacket.packet.nmeaTime, token);
            break;
          case 4:   // nmea latitude in format ddmm.mmmmm
            if(strlen(token) != 10)
            {
              return;
            }
            strcpy(gpsPacket.packet.gpsLatitude, token);
            break;
          case 6:   // nmea longitude in format dddmm.mmmmm
            if(strlen(token) != 11)
            {
              return;
            }
            strcpy(gpsPacket.packet.gpsLongitude, token);
            break;
          default:  // ignore all others
            break;
        }
        token = strtok(NULL,",");
      }
      // bad data packet
      if((tokenCount != 11) ||
         (strcmp(gpsPacket.packet.nmeaTime, "000000.000") == 0))
      {
        return;
      }
      sendData();
    }
    // is connection still alive? if not retry
    else
    {
      // 30 seconds since last message recieved, retry
      if(currentSampleTime - lastSampleTime > DISCONNECT_RETRY)
      {
        #ifdef PRINT_DEBUG
        Serial.print(DISCONNECT_RETRY/1000000); Serial.println(" seconds since last message or reconnect attempt, reattempt GPS connection");
        #endif
        ConnectToGPS();
        lastSampleTime = currentSampleTime;
      }
    }
  }
  // not logging - every 10000000 micros (10 sec) send heartbeat
  // triggers a timestamp send from hub
  else
  {
    lastSampleInterval = currentSampleTime - lastSampleTime; 
    if(lastSampleInterval >= 10000000)
    {
      SendHeartBeat();
      lastSampleTime = currentSampleTime;
    }
  }
}
//
// send data
//
void sendData()
{
  uint8_t result = esp_now_send(hub_addr, &gpsPacket.dataBytes[0], sizeof(gpsPacket));
  sentCount++;
  #ifdef PRINT_DEBUG
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.print(gpsPacket.packet.leafType);
  Serial.print(" Time ");
  Serial.print(gpsPacket.packet.timeStamp);
  Serial.print(" Values GPS nmeaTime ");
  Serial.print(gpsPacket.packet.nmeaTime); Serial.print(" LAT ");
  Serial.print(gpsPacket.packet.gpsLatitude); Serial.print(" LONG ");
  Serial.println(gpsPacket.packet.gpsLongitude);
  #endif
  if(result != 0)
  {
    errorCount++;
    Serial.print("send error");
    Serial.print(errorCount);
    Serial.print(" of ");
    Serial.print(sentCount);
    Serial.println(" send attempts");
  }
}
//
// send heartbeat
//
void SendHeartBeat()
{
  // was disconnected from hub, try to reconnect
  if(!esp_now_is_peer_exist(hub_addr))
  {
    #ifdef PRINT_DEBUG
    Serial.println("Reconnecting");
    #endif
    esp_now_add_peer((const esp_now_peer_info_t*)hub);
  }
  timeStamp.packet.msgType[0] = 'H';
  timeStamp.packet.timeStamp = micros() - timestampAdjust;
  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(timeStamp));
  #ifdef PRINT_DEBUG
  Serial.print(micros());
  Serial.print(" To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.print(timeStamp.packet.msgType);
  Serial.print(" Time ");
  Serial.print(timeStamp.packet.timeStamp);
  Serial.print(" To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.println();
  #endif
}
//
// request timestamp from hub
//
void requestTimestamp()
{
  timeStamp.packet.msgType[0] = 'T';
  timeStamp.packet.timeStamp = micros() - timestampAdjust;
  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(timeStamp));
  #ifdef PRINT_DEBUG
  Serial.print(timeStamp.packet.timeStamp);
  Serial.print("To ");
  Serial.print(hub_addr[0], HEX);
  for(int idx = 1; idx < 6; idx++)
  {
    Serial.print(":");
    Serial.print(hub_addr[idx], HEX);
  }
  Serial.print(" Type ");
  Serial.println((char)timeStamp.packet.msgType[0]);
  if(result != 0)
  {
    Serial.print("message send error ");
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
    #ifdef PRINT_DEBUG
    Serial.println("ESPNow Init Success");
    #endif
  }
  else 
{
    #ifdef PRINT_DEBUG
    Serial.println("ESPNow Init Failed");
    #endif
    // restart
    ESP.restart();
  }
}
//
// callback when data is sent
//
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
if(status != 0)
  {
    #ifdef PRINT_DEBUG
    char macStr[18];
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
//
// callback when data is received
//
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  if(data[0] == 'T')
  {
    memcpy(&timeStamp, data, sizeof(timeStamp));
    timestampAdjust =  micros() - timeStamp.packet.timeStamp;
    #ifdef PRINT_DEBUG
    Serial.print(mac_addr[0], HEX);
    for(int idx = 1; idx < 6; idx++)
    {
      Serial.print(":");
      Serial.print(mac_addr[idx], HEX);
    }
    unsigned long localTs = micros();
    Serial.print(" LOCAL timestamp ");
    Serial.print (localTs);
    Serial.print(" HUB timestamp ");
    Serial.print (timeStamp.packet.timeStamp);
    Serial.print(" adjustment ");
    Serial.print ((long)timestampAdjust);
    Serial.print(" corrected ");
    Serial.println(localTs - timestampAdjust);
    #endif
  }
  // change state of logging
  else if(data[0] == 'B')
  {
    #ifdef PRINT_DEBUG
    Serial.println("START Logging");
    #endif
    isLogging = true;
  }
  else if(data[0] == 'E')
  {
    #ifdef PRINT_DEBUG
    Serial.println("END Logging");
    #endif
    isLogging = false;
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
    #ifdef PRINT_DEBUG
    Serial.println("Connected Succesfully!");
    #endif
  }
  else 
  {
    while(!SerialBT.connected(10000)) 
    {
      #ifdef PRINT_DEBUG
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
      #endif
    }
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) 
  {
    #ifdef PRINT_DEBUG
    Serial.println("Disconnected Succesfully!");
    #endif
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
}
