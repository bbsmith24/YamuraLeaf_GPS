//
// Yamura Log data structures
// BBS 3/2021
//
// structures for data from sensors - don't use directly
// unions for sending data to peers - maps structure data to corresponding byte array
// Note: __attribute__((__packed__)) eliminates alignment gaps
//
// Packet Type codes
// T - timestamp request/update (leaf requests, hub updates)
// H - heartbeat
// I - IMU (accel + gyro + mag)
// C - combined digital and A2D input
// D - digital only input
// A - analog only input
// G - GPS input
//
// B - begin logging
// E - end logging
//
#define LOGGING_BEGIN 'B'
#define LOGGING_END 'E'
#define TIMESTAMP_TYPE 'T'
#define HEARTBEAT_TYPE 'H'
#define TIMEPACKET_SIZE 5
struct TimeStamp
{
  char msgType            __attribute__((__packed__));    // 1 bytes  - type, in this case 'T' for timestamp request, 'H' for heartbeat, 'B' to start logging, 'E' to end logging
  unsigned long timeStamp __attribute__((__packed__));    // 4 bytes - millis() value of sample
  //                                                         5 bytes total 
};
union TimeStampPacket
{
  TimeStamp packet;
  uint8_t dataBytes[TIMEPACKET_SIZE];
};
void PrintTimestamp(unsigned long local, TimeStampPacket info)
{
  #ifdef PRINT_DEBUG
  Serial.print("Type\t");
  Serial.print(info.packet.msgType);
  Serial.print("\tLocal\t");
  Serial.print(local);
  Serial.print("\tTimestamp\t");
  Serial.println(info.packet.timeStamp);
  #endif
}
//
#define IMU_LEAFTYPE 'I'
#define IMUPACKET_SIZE 17
struct IMULeaf
{
  char leafType           __attribute__((__packed__));   //  1 byte  - type, in this case 'I' for IMU
  unsigned long timeStamp __attribute__((__packed__));   //  4 bytes - millis() value of sample
  int16_t values[6]       __attribute__((__packed__));   // 12 bytes - X,Y,Z acceleration values and i, j, k rotation values
  //                                                        17 bytes total 
};
union IMUPacket
{
	IMULeaf packet;
	uint8_t dataBytes[IMUPACKET_SIZE];
};
void PrintIMU(IMUPacket info)
{
  #ifdef PRINT_DEBUG
  Serial.print("Timestamp\t");
  Serial.print(info.packet.timeStamp);
  Serial.print("\tACCEL X\t");
  Serial.print(info.packet.values[0]);
  Serial.print("\tY\t");
  Serial.print(info.packet.values[1]);
  Serial.print("\tZ\t");
  Serial.print(info.packet.values[2]);
  Serial.print("\tGYRO X\t");
  Serial.print(info.packet.values[3]);
  Serial.print("\tY\t");
  Serial.print(info.packet.values[4]);
  Serial.print("\tZ\t");
  Serial.println(info.packet.values[5]);
  #endif
}
//
#define COMBINEDIO_LEAFTYPE 'C'
#define COMBINEDIOPACKET_SIZE 15
struct CombinedIOLeaf
{
  char leafType           __attribute__((__packed__));    // 1 byte,  type, in this case 'C' for combined IO
  unsigned long timeStamp __attribute__((__packed__));    // 4 bytes, millis() value of sample
  int16_t a2dValues[4]    __attribute__((__packed__));    // 8 bytes, 2 bytes per a2d channel
  int16_t digitalValue    __attribute__((__packed__));    // 2 byte,  1 bit per digital channel
  //                                                        15 bytes total 
};
union CombinedIOPacket
{
	CombinedIOLeaf packet;
	uint8_t dataBytes[COMBINEDIOPACKET_SIZE];
};
void PrintCombined(CombinedIOPacket info)
{
  #ifdef PRINT_DEBUG
  Serial.print("Timestamp\t");
  Serial.print(info.packet.timeStamp);
  for(int idx = 0; idx < 4; idx++)
  {
    Serial.print("\tA");
    Serial.print(idx);
    Serial.print("\t");
    Serial.print(info.packet.a2dValues[idx]);
  }
  Serial.println(info.packet.digitalValue, HEX);
  #endif
}
#define DIGITAL_LEAFTYPE 'D'
#define DIGITALPACKET_SIZE 7
struct DigitalLeaf
{
  char leafType           __attribute__((__packed__));    // 1 byte,  type, in this case 'D' for digital (on/off) channels
  unsigned long timeStamp __attribute__((__packed__));    // 4 bytes, millis() value of sample
  int16_t digitalValue    __attribute__((__packed__));    // 2 byte,  1 bit per digital channel
  //                                                         7 bytes total 
};
union DigitalPacket
{
  DigitalLeaf packet;
  uint8_t dataBytes[DIGITALPACKET_SIZE];
};
void PrintDigital(DigitalPacket info)
{
  #ifdef PRINT_DEBUG
  Serial.print("Timestamp\t");
  Serial.print(info.packet.timeStamp);
  Serial.println(info.packet.digitalValue, HEX);
  #endif
}

#define ANALOG_LEAFTYPE 'A'
#define ANALOGPACKET_SIZE 13
struct AnalogLeaf
{
  char leafType           __attribute__((__packed__));    // 1 byte,  type, in this case 'D' for digital (on/off) channels
  unsigned long timeStamp __attribute__((__packed__));    // 4 bytes, millis() value of sample
  int16_t a2dValues[4]    __attribute__((__packed__));    // 8 bytes, 2 per a2d channel
  //                                                        13 bytes total 
};
union AnalogPacket
{
  AnalogLeaf packet;
  uint8_t dataBytes[ANALOGPACKET_SIZE];
};
//
//
//
void PrintAnalog(AnalogPacket info)
{
  #ifdef PRINT_DEBUG
  Serial.print("Timestamp\t");
  Serial.print(info.packet.timeStamp);
  for(int idx = 0; idx < 4; idx++)
  {
    Serial.print("\tA");
    Serial.print(idx);
    Serial.print("\t");
    Serial.print(info.packet.a2dValues[idx]);
  }
  Serial.println();
  #endif
}
#define GPS_LEAFTYPE 'G'
#define GPSPACKET_SIZE 44
struct GPSLeaf
{
  char leafType           __attribute__((__packed__));    // 1 byte  - type, in this case 'G' for GPS
  unsigned long timeStamp __attribute__((__packed__));    // 4 bytes - micros() value of sample
  uint8_t gpsDay          __attribute__((__packed__));    // 1 byte of GPS day
  uint8_t gpsMonth        __attribute__((__packed__));    // 1 byte of GPS month
  uint8_t gpsHour         __attribute__((__packed__));    // 1 byte of GPS hour
  uint8_t gpsMinute       __attribute__((__packed__));    // 1 byte of GPS minute
  uint8_t gpsSecond       __attribute__((__packed__));    // 1 byte of GPS second
  uint8_t gpsCentisecond  __attribute__((__packed__));    // 1 byte of GPS centisecond
  double gpsLatitude      __attribute__((__packed__));    // 8 bytes of GPS latitude
  double gpsLongitude     __attribute__((__packed__));    // 8 bytes of GPS longitude
  double gpsSpeed         __attribute__((__packed__));    // 8 bytes of GPS speed
  double gpsCourse        __attribute__((__packed__));    // 8 bytes of GPS course (degrees)
  uint8_t gpsSIV          __attribute__((__packed__));    // 1 byte of satellites in view
  //                                                        44 bytes total 
};
union GPSPacket
{
  GPSLeaf packet;
  uint8_t dataBytes[GPSPACKET_SIZE];
};
//
//
//
void PrintGPS(GPSPacket info)
{
  #ifdef PRINT_DEBUG
  Serial.print("Timestamp\t");
  Serial.print(info.packet.timeStamp);
  Serial.print("\tGPS time\t");
  Serial.print(info.packet.gpsHour);
  Serial.print(":");
  Serial.print(info.packet.gpsMinute);
  Serial.print(":");
  Serial.print(info.packet.gpsSecond);
  Serial.print(".");
  Serial.print(info.packet.gpsCentisecond);
  Serial.print("\tDate\t");
  Serial.print(info.packet.gpsMonth);
  Serial.print("/");
  Serial.print(info.packet.gpsDay);
  Serial.print("\tLAT\t");
  Serial.print(info.packet.gpsLatitude);
  Serial.print("\tLONG\t");
  Serial.print(info.packet.gpsLongitude);
  Serial.print("\tMPH\t");
  Serial.print(info.packet.gpsSpeed);
  Serial.print("\tCourse\t");
  Serial.print(info.packet.gpsCourse);
  Serial.print("\tSIV\t");
  Serial.println(info.packet.gpsSIV);
  #endif
}
