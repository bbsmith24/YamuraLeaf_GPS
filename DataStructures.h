//
// Yamura Log data structures
// BBS 3/2021
//
// structures for data from sensors - don't use directly
// unions for sending data to peers - maps structure data to corresponding byte array
//
#define TIMEPACKET_SIZE 8
struct TimeStamp
{
  char msgType[4];          // 4 bytes
  unsigned long timeStamp;  // 4 bytes - millis() value of sample
};
union TimeStampPacket
{
  TimeStamp packet;
  uint8_t dataBytes[TIMEPACKET_SIZE];
};
//
#define ACCELPACKET_SIZE 20
struct AccelLeaf
{
  char leafType[4];            //  4 byte  - type, in this case 'A' for accelerometer
  unsigned long timeStamp;     //  4 bytes - millis() value of sample
  int16_t values[6];           // 12 bytes - X,Y,Z acceleration values and i, j, k rotation values
};
union AccelPacket
{
	AccelLeaf packet;
	uint8_t dataBytes[ACCELPACKET_SIZE];
};
//
#define IOPACKET_SIZE 20
struct IOLeaf
{
  char leafType[4];         // 4 byte,  type, in this case 'I' for IO - might need to make this 2 characters...
  unsigned long timeStamp;  // 4 bytes, millis() value of sample
  int16_t a2dValues[4];     // 8 bytes, 2 per a2d channel
  int16_t digitalValue;     // 2 byte,  1 bit per digital channel
  char pad[2];              // 2 bytes, pad to fill out to multiple of 4 bytes
};
union IOPacket
{
	IOLeaf packet;
	uint8_t dataBytes[IOPACKET_SIZE];
};
//
#define GPSPACKET_SIZE 56
struct GPSLeaf
{
  char leafType[4];           //  4 bytes - type, in this case 'G' for GPS
  unsigned long timeStamp;    //  4 bytes - micros() value of sample
  char nmeaTime[16];          // 16 bytes of nmea time string in form hhmmss.sss
  char gpsLatitude[16];       // 16 bytes of nmea latitude in form ddmm.mmmm              
  char gpsLongitude[16];      // 16 bytes of nmea longitude in form dddmm.mmmm              
};
union GPSPacket
{
	GPSLeaf packet;
	uint8_t dataBytes[GPSPACKET_SIZE];
};
