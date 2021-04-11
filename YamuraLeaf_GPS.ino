
/*
   Yamura Leaf - bluetooth GPS for Dual XGPS160
   BBS 3/2021
   send GPS position info to hub

   for ESP32 with built in bluetooth module
*/
#define PRINT_DEBUG
#define TARGET_INTERVAL 50000
#define TIMESTAMP_REQUEST_INTERVAL 5000000
#define DISCONNECT_RETRY 10000000 // loss of BT for GPS unit


#include <esp_now.h>
#include <WiFi.h>
#include "BluetoothSerial.h"
#include "DataStructures.h"
#include <TinyGPS++.h>
// timestamp (H and T types)
//TimeStampPacket timeStamp;
// digital/A2D data (I type)
//GPSPacket gpsPacket;

char gpsRecieved[128];

unsigned long lastSampleTime;
unsigned long currentSampleTime;
unsigned long targetInterval = TARGET_INTERVAL;
unsigned long lastSampleInterval = 0;
unsigned long timestampAdjust = 0;
unsigned long lastHeartbeatTime = 0;
bool isLogging = false;
unsigned long sentCount = 0;
unsigned long errorCount = 0;
// hub MAC addres
uint8_t hub_addr[] = { 0x7C, 0x9E, 0xBD, 0x30, 0x54, 0xCC};
esp_now_peer_info *hub;

BluetoothSerial SerialBT;
String btName = "XGPS160-45E134";
char *btPin = "1234"; //<- standard pin would be provided by default
bool btConnected;
TinyGPSPlus gpsParser;
int gpsHour;
int gpsMinute;
int gpsSecond;
int gpsCentiSecond;

//#define ESP8266_LED 5
//
//
//
//
//
//
xQueueHandle  gpsSendQueue;
xQueueHandle  timeStampSendQueue;
xQueueHandle  timeStampReceiveQueue;

TaskHandle_t gpsSendTask;
TaskHandle_t timeStampSendTask;
TaskHandle_t timeStampReceiveTask;
void setup()
{
  Serial.begin(115200);
  int blinkState = HIGH;
  //pinMode(ESP8266_LED, OUTPUT); // built in LED
  
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  // This is the mac address of this device
  Serial.println();
  Serial.print("YamuraLeaf GPS ");Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();

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
 // gpsPacket.packet.leafType = GPS_LEAFTYPE;

  #ifdef PRINT_DEBUG
  Serial.println("Begin bluetooth initialization");
  // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
  // to resolve name to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices bluetooth address and device names
  Serial.print("Attempt to connect to "); Serial.println(btName);
  #endif
  SerialBT.begin("GPSLeaf", true); 
  ConnectToGPS();
  //digitalWrite(ESP8266_LED, LOW); // built in LED
  #ifdef PRINT_DEBUG
  Serial.println();
  #endif
  //for(int cnt = 0; cnt < 30; cnt++)
  //{
  //    digitalWrite(ESP8266_LED, blinkState);
  //    delay(100);
  //    blinkState = blinkState == LOW ? blinkState = HIGH :blinkState = LOW; 
  //}
  currentSampleTime = micros();
  lastSampleTime = currentSampleTime;
  // create a queue for sensor readings and timestamps to send
  // send function will pull from here to send to hub
  gpsSendQueue = xQueueCreate(100, sizeof(GPSPacket));
 if(gpsSendQueue == 0)
  {
    Serial.println("Error creating gpsSendQueue");
    while(true) {}
  } 
  Serial.println("Created gpsSendQueue");
  // create a queue for timestamps to send
  // send function pulls messages to send to hub here
  timeStampSendQueue = xQueueCreate(100, sizeof(TimeStampPacket));
  if(timeStampSendQueue == 0)
  {
    Serial.println("Error creating timeStampSendQueue");
    while(true) {}
  }
  Serial.println("Created timeStampSendQueue");
  // create a queue for timestamps received
  // receive function put messages received from hub here
  timeStampReceiveQueue = xQueueCreate(100, sizeof(TimeStampPacket));
  if(timeStampReceiveQueue == 0)
  {
    Serial.println("Error creating timeStampReceiveQueue");
    while(true) {}
  }
   Serial.println("Created timeStampReceiveQueueeue");
  //
  // start the ESPNOW send tasks - one for GPS data, one for timestamp/heartbeat data
  //
  xTaskCreatePinnedToCore(
              SendGPSDataTask, 
              "SendGPS", 
              10000, 
              NULL,
              0,
              &gpsSendTask,
              0);
  xTaskCreatePinnedToCore(
              SendTimestampDataTask,    // function
              "SendTimestamp",          // name
              10000,                    // stack size
              NULL,                     // parameter passed
              0,                        // priority 0=low
              &timeStampSendTask,       // task handle
              0);                       // core
  Serial.println("Created SendTimestampDataTask");
  xTaskCreatePinnedToCore(
              ReceiveTimestampDataTask,
              "ReceiveTimestamp", 
              10000, 
              NULL,
              0,
              &timeStampReceiveTask,
              0);

   Serial.println("Created ReceiveTimestampDataTask");
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // register for receive callback for data send response
  esp_now_register_recv_cb(OnDataRecv);
  // send message callback
  esp_now_register_send_cb(OnDataSent);
  Serial.println("Registered send/receive callbacks");
  Serial.println("Running");
}
//
// read GPS, push data to hub
// in this case, we're waiting for data from GPS unit, so just send it when it's ready
//
void loop() 
{
  currentSampleTime = micros();
  // get message from bluetooth
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
      gpsParser.encode(inByte);
      gpsRecieved[charIdx]=(char)inByte;
      charIdx++;
    }
    // invalid time or position
    if((!gpsParser.time.isValid()) ||
       (!gpsParser.location.isValid()) ||
       (!gpsParser.location.isUpdated()))
    {
      return;
    }
    //Serial.print("\tGPS data received - ");
    int curHour = gpsParser.time.hour();
    int curMinute = gpsParser.time.minute();
    int curSecond = gpsParser.time.second();
    int curCentiSecond = gpsParser.time.centisecond();
    // duplicate timestamp
    if((gpsHour == curHour) &&
       (gpsMinute == curMinute) &&
       (gpsSecond == curSecond) &&
       (gpsCentiSecond == curCentiSecond))
    {
      return;
    }
    //
    GPSPacket gpsInfo;
    gpsInfo.packet.leafType = GPS_LEAFTYPE;
    gpsInfo.packet.timeStamp = lastSampleTime;
    gpsInfo.packet.gpsDay = gpsParser.date.day();
    gpsInfo.packet.gpsMonth = gpsParser.date.month();
    gpsInfo.packet.gpsHour = curHour;
    gpsInfo.packet.gpsMinute = curMinute;
    gpsInfo.packet.gpsSecond = curSecond;
    gpsInfo.packet.gpsCentisecond = curCentiSecond;
    gpsInfo.packet.gpsLatitude = gpsParser.location.lat();
    gpsInfo.packet.gpsLongitude = gpsParser.location.lng();
    gpsInfo.packet.gpsSpeed = gpsParser.speed.mph();
    gpsInfo.packet.gpsCourse = gpsParser.course.deg();
    gpsInfo.packet.gpsSIV = (uint8_t)gpsParser.satellites.value();
    // update last gps time sent
    gpsHour = curHour;
    gpsMinute = curMinute;
    gpsSecond = curSecond;
    gpsCentiSecond = curCentiSecond;
    #ifdef PRINT_DEBUG
    PrintGPS(gpsInfo);
    #endif
    if(isLogging)
    {
      xQueueSendToBack( gpsSendQueue, &gpsInfo, portMAX_DELAY );
    }
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
      return;
    }
  }
  // not logging anf GPS alive - every 10000000 micros (10 sec) send heartbeat
  // triggers a timestamp send from hub
  if(!isLogging)
  {
    lastSampleInterval = currentSampleTime - lastHeartbeatTime; 
    if(lastSampleInterval >= 10000000)
    {
      Serial.print("Adding HEARTBEAT to timeStampSendQueue - queue length is "); 
      TimeStampPacket timeStampHeartbeat;
      timeStampHeartbeat.packet.msgType = HEARTBEAT_TYPE;
      timeStampHeartbeat.packet.timeStamp = micros() - timestampAdjust;
      xQueueSendToBack( timeStampSendQueue, &timeStampHeartbeat,0);// portMAX_DELAY );
      Serial.println(uxQueueMessagesWaiting(timeStampSendQueue));
      lastHeartbeatTime = micros();
    }
  }
}
//
//
//
void SendGPSDataTask(void * pvParameters )
{
  while(true)
  {
    if(uxQueueMessagesWaiting(gpsSendQueue) == 0)
    {
      vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    else
    {
      GPSPacket gpsInfoToSend;
      if (xQueueReceive( gpsSendQueue, &gpsInfoToSend, pdMS_TO_TICKS( 10 ) ) == pdPASS)
      {
        uint8_t result = esp_now_send(hub_addr, &gpsInfoToSend.dataBytes[0], sizeof(GPSPacket));
        Serial.print("Message sent from GPS SEND queue with result "); Serial.print(result);
        Serial.print(" GPS SEND queue has ");Serial.print(uxQueueMessagesWaiting(gpsSendQueue));Serial.println(" entries");
      }
      else
      {
        Serial.print("Error unloading data from GPS SEND queue - queue has ");
        Serial.print(uxQueueMessagesWaiting(gpsSendQueue));Serial.println(" entries");
      }
    }
  }
}
//
//
//
void SendTimestampDataTask(void * pvParameters )
{
  while(true)
  {
    if(uxQueueMessagesWaiting(timeStampSendQueue) == 0)
    {
          vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    else
    {
      TimeStampPacket timeStampToSend;
      if (xQueueReceive( timeStampSendQueue, &timeStampToSend, pdMS_TO_TICKS( 10 ) ) == pdPASS)
      {
        uint8_t result = esp_now_send(hub_addr, &timeStampToSend.dataBytes[0], sizeof(TimeStampPacket));
        Serial.print("Message sent from TIMESTAMP SEND queue with result ");Serial.print(result);
        Serial.print(" TIMESTAMP SEND queue has ");Serial.print(uxQueueMessagesWaiting(timeStampSendQueue));Serial.println(" entries");
      }
      else
      {
        Serial.println("Error unloading data from TIMESTAMP SEND queue - queue has ");
        Serial.print(uxQueueMessagesWaiting(timeStampSendQueue));Serial.println(" entries");
      }
    }
  }
}
void ReceiveTimestampDataTask(void * pvParameters )
{
  while(true)
  {
    if(uxQueueMessagesWaiting(timeStampReceiveQueue) == 0)
    {
          vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    else
    {
      TimeStampPacket timeStampReceived;
      if (xQueueReceive( timeStampReceiveQueue, &timeStampReceived, pdMS_TO_TICKS( 10 ) ) == pdPASS)
      {
        Serial.print("Message received from TIMESTAMP RECEIVE queue - queue has ");
        Serial.print(uxQueueMessagesWaiting(timeStampReceiveQueue));Serial.println(" entries");
        // get local timestamp adjustment
        timestampAdjust =  micros() - timeStampReceived.packet.timeStamp;
        #ifdef PRINT_DEBUG
        PrintTimestamp(micros(), timeStampReceived);
        #endif
      }
      else
      {
        Serial.println("Error unloading data from TIMESTAMP RECEIVE queue - queue has ");
        Serial.print(uxQueueMessagesWaiting(timeStampReceiveQueue));Serial.println(" entries");
      }
    }
  }
}
//
// send data
//
//void sendData()
//{
//  uint8_t result = esp_now_send(hub_addr, &gpsPacket.dataBytes[0], sizeof(GPSPacket));
//  sentCount++;
//  #ifdef PRINT_DEBUG
//  Serial.print("To ");
//  Serial.print(hub_addr[0], HEX);
//  for(int idx = 1; idx < 6; idx++)
//  {
//    Serial.print(":");
//    Serial.print(hub_addr[idx], HEX);
//  }
//  Serial.print(" Type ");
//  Serial.print(gpsPacket.packet.leafType);
//  Serial.print(" Time ");
//  Serial.print(gpsPacket.packet.timeStamp);
//  Serial.print(" GPS Time ");
//  Serial.print(gpsPacket.packet.gpsHour); Serial.print(":");Serial.print(gpsPacket.packet.gpsMinute); Serial.print(":");Serial.print(gpsPacket.packet.gpsSecond);Serial.print(".");Serial.print(gpsPacket.packet.gpsCentisecond);
//  Serial.print(" LAT ");
//  Serial.print(gpsPacket.packet.gpsLatitude); Serial.print(" LONG ");
//  Serial.println(gpsPacket.packet.gpsLongitude);
//  #endif
//  if(result != 0)
//  {
//    errorCount++;
//    Serial.print("send error");
//    Serial.print(errorCount);
//    Serial.print(" of ");
//    Serial.print(sentCount);
//   Serial.println(" send attempts");
//  }
//}
//
// send heartbeat
//
//void SendHeartBeat()
//{
//  // was disconnected from hub, try to reconnect
//  if(!esp_now_is_peer_exist(hub_addr))
//  {
//    #ifdef PRINT_DEBUG
//    Serial.println("Reconnecting");
//    #endif
//    esp_now_add_peer((const esp_now_peer_info_t*)hub);
//  }
//  timeStamp.packet.msgType = HEARTBEAT_TYPE;
//  timeStamp.packet.timeStamp = micros() - timestampAdjust;
//  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(TimeStampPacket));
//  #ifdef PRINT_DEBUG
//  Serial.print(micros());
//  Serial.print(" To ");
//  Serial.print(hub_addr[0], HEX);
//  for(int idx = 1; idx < 6; idx++)
//  {
//    Serial.print(":");
//    Serial.print(hub_addr[idx], HEX);
//  }
//  Serial.print(" Type ");
//  Serial.print(timeStamp.packet.msgType);
//  Serial.print(" Time ");
//  Serial.print(timeStamp.packet.timeStamp);
//  Serial.print(" To ");
//  Serial.print(hub_addr[0], HEX);
//  for(int idx = 1; idx < 6; idx++)
//  {
//    Serial.print(":");
//    Serial.print(hub_addr[idx], HEX);
//  }
//  Serial.println();
//  #endif
//}
//
// request timestamp from hub
//
//void requestTimestamp()
//{
//  timeStamp.packet.msgType = TIMESTAMP_TYPE;
//  timeStamp.packet.timeStamp = micros() - timestampAdjust;
//  uint8_t result = esp_now_send(hub_addr, &timeStamp.dataBytes[0], sizeof(TimeStampPacket));
//  #ifdef PRINT_DEBUG
//  Serial.print(timeStamp.packet.timeStamp);
//  Serial.print("To ");
//  Serial.print(hub_addr[0], HEX);
//  for(int idx = 1; idx < 6; idx++)
//  {
//    Serial.print(":");
//    Serial.print(hub_addr[idx], HEX);
//  }
//  Serial.print(" Type ");
//  Serial.println((char)timeStamp.packet.msgType[0]);
//  if(result != 0)
//  {
//    Serial.print("message send error ");
//  }
//  #endif
//}
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
  if((data[0] == TIMESTAMP_TYPE) && (data_len == sizeof(TimeStampPacket)))
  {
    //memcpy(&timeStamp.dataBytes, data, sizeof(TimeStampPacket));
    xQueueSendToBack( timeStampReceiveQueue, data, 0);// portMAX_DELAY );
    Serial.print("Added TIMESTAMP to TIMESTAMP RECEIVE queue - queue has "); 
    Serial.print(uxQueueMessagesWaiting(timeStampReceiveQueue));Serial.println(" entries");
  }
  // change state of logging
  else if(data[0] == LOGGING_BEGIN)
  {
    #ifdef PRINT_DEBUG
    Serial.println("START Logging");
    #endif
    isLogging = true;
  }
  else if(data[0] == LOGGING_END)
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
  //if (SerialBT.disconnect()) 
  //{
  //  #ifdef PRINT_DEBUG
  //  Serial.println("Disconnected Succesfully!");
  //  #endif
  //}
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  //SerialBT.connect();
  //while(!SerialBT.connected(10000)) 
  //{
  //  #ifdef PRINT_DEBUG
  //  Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
  //  #endif
  //  btConnected = false;
  //}
  //btConnected = true;
  #ifdef PRINT_DEBUG
  Serial.println("Connected Succesfully!");
  #endif
}
