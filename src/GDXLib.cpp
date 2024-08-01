/* This is a library for using Vernier Go Direct sensors
 with Arduinos that have support in the Arduino BLE library
 
Version  - 
---  
*/
//#define DEBUG //NOTE THIS PRINTS OUT DECODING STUFF!!!
#include "ArduinoBLE.h"
#include "Arduino.h"
#include "GDXLib.h"
#define GDXLib_LIB_VERSION "0.88"//automatically displayed

GDXLib::GDXLib()
{}
char deviceName[32];
int channelNumber;
uint8_t chargerStatus;
int batteryPercent;
//char strBuffer[32];
//char strFW1[16];// These are not used in my code
//char strFW2[16];
#define D2PIO_CMD_ID_GET_STATUS                          0x10
//#define D2PIO_CMD_ID_START_MEASUREMENTS                  0x18
//#define D2PIO_CMD_ID_STOP_MEASUREMENTS                   0x19
//#define D2PIO_CMD_ID_INIT                                0x1A
//#define D2PIO_CMD_ID_SET_MEASUREMENT_PERIOD              0x1B
//#define D2PIO_CMD_ID_CLEAR_ERROR_FLAGS                   0x34
#define D2PIO_CMD_ID_GET_SENSOR_CHANNEL_INFO             0x50
//#define D2PIO_CMD_ID_GET_SENSOR_CHANNELS_AVAILABLE_MASK  0x51
//#define D2PIO_CMD_ID_DISCONNECT                          0x54
//#define D2PIO_CMD_ID_GET_DEVICE_INFO                     0x55
//#define D2PIO_CMD_ID_GET_SENSOR_CHANNELS_DEFAULT_MASK    0x56
//#define D2PIO_CMD_ID_IDENTIFY                            0x58
//#define D2PIO_CMD_ID_GET_BATTERY_LEVEL                   0x78

//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL                   0
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_DIGITAL        1
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_ANALOG         2
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_MULTIPLE_PERIODS         3
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_PERIODIC_DIGITAL_COUNTS  4
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL_SIGNED            5
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL_REAL32            6
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_WIDE_REAL32              7
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_SINGLE_CHANNEL_REAL32    8
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_SINGLE_CHANNEL_INT32     9
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_REAL32         10
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_INT32          11
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NEXT_PERIODIC_TIMESTAMP  12
//#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_DROPPED_MEASUREMENT      13

// #define D2PIO_MASK_STATUS_ERROR_CMD_NOT_RECOGNIZED      0x01
// #define D2PIO_MASK_STATUS_ERROR_CMD_IGNORED             0x02
// #define D2PIO_MASK_STATUS_ADC_UNCALIBRATED              0x04
// #define D2PIO_MASK_STATUS_AUTOID_UNCALIBRATED           0x08
// #define D2PIO_MASK_STATUS_ERROR_INTERNAL_ERROR1         0x10
// #define D2PIO_MASK_STATUS_ERROR_AUDIO_CONTROL_FAILURE   0x10
// #define D2PIO_MASK_STATUS_ERROR_INTERNAL_ERROR2         0x20
// #define D2PIO_MASK_STATUS_ERROR_AUDIO_STREAM_FAILURE    0x20
// #define D2PIO_MASK_STATUS_ERROR_MASTER_FIFO_OVERFLOW    0x40
// #define D2PIO_MASK_STATUS_ERROR_DIGITAL_TRANSITION_LOST 0x80

// #define D2PIO_CHARGER_STATE_IDLE     0
// #define D2PIO_CHARGER_STATE_CHARGING 1
// #define D2PIO_CHARGER_STATE_COMPLETE 2
// #define D2PIO_CHARGER_STATE_ERROR    3

//#define D2PIO_MAX_ORDERCODE_LENGTH 16
//#define D2PIO_MAX_SERIALNUMBER_LENGTH 16
//#define D2PIO_BLE_ADDRESS_LENGTH 6
//#define D2PIO_MAX_DEVICENAME_LENGTH 32
//#define D2PIO_MAX_DEVICEDESCRIPTION_LENGTH 64
#define D2PIO_MAX_NUM_BYTES_IN_SENSOR_DESCRIPTION 60
#define D2PIO_MAX_NUM_BYTES_IN_SENSOR_UNIT 32
struct D2PIOGetStatusCmdResponsePayload
{
  uint8_t  status; //See D2PIO_MASK_STATUS_*.
  uint8_t  spare;
  uint8_t  majorVersionMasterCPU;
  uint8_t  minorVersionMasterCPU;
  uint16_t buildNumMasterCPU;
  uint8_t  majorVersionSlaveCPU;
  uint8_t  minorVersionSlaveCPU;
  uint16_t buildNumSlaveCPU;
  uint8_t  batteryLevelPercent;
  uint8_t  chargerState;       //D2PIO_CHARGER_STATE_*
} __attribute__ ((packed));

/*
struct D2PIOStartMeasurementsParams
{
  int8_t   channel;  // -1 => all channels specified by channelBits.
  uint8_t  sampleAtTime0flag;
  uint32_t channelBits;    //Id's channels that should start taking measurements. Ignored unless (channel == -1).
  int64_t  startTimestamp; //Microseconds since Jan 1 1970 Coordinated Universal Time(aka UTC aka Greenwich Mean Time), (timestamp == 0) => start immediately.
} __attribute__ ((packed));

struct D2PIOStopMeasurementsParams
{
  int8_t   channel;  // -1 => all channels specified by channelBits. 
  uint8_t  spare;
  uint32_t channelBits;    //Id's channels that should start taking measurements. Ignored unless (channel == -1).
} __attribute__ ((packed));

struct D2PIOSetMeasurementPeriodParams
{
  int8_t   channel;  // -1 => all channels specified by channelBits.
  uint8_t  spare;
  uint64_t measurementPeriod; //In 'ticks', typically a tick == 1 microsecond
} __attribute__ ((packed));
*/

struct D2PIOGetSensorChannelInfoCmdResponse
{
  int8_t   channel;
  uint8_t  spare;
  uint32_t sensorId; //unique record key for the Sensor Map
  uint8_t  numericMeasType;    //D2PIO_NUMERIC_MEAS_TYPE_...
  uint8_t  samplingMode;       //D2PIO_SAMPLING_MODE_...
  char     sensorDescription[D2PIO_MAX_NUM_BYTES_IN_SENSOR_DESCRIPTION];
  char     sensorUnit[D2PIO_MAX_NUM_BYTES_IN_SENSOR_UNIT];
  double   measurementUncertainty; //real number expressed in sensorUnit's.
  double   minMeasurement;     //real number expressed in sensorUnit's.
  double   maxMeasurement;     //real number expressed in sensorUnit's.
  uint32_t minMeasurementPeriod;   //in microsecond ticks
  uint64_t maxMeasurementPeriod;   //in microsecond ticks
  uint32_t typMeasurementPeriod;   //in microsecond ticks
  uint32_t measurementPeriodGranularity; //in microsecond ticks, supported periods are integer multiples of measurementPeriodGranularity
  uint32_t mutualExclusionMask; //channels that cannot be enabled at the same time as this channel
} __attribute__ ((packed));

//the line below specifies the threshold RSSI for connecting!!!
// #define GDX_BLE_AUTO_CONNECT_RSSI_THRESHOLD -44
// #define GDX_BLE_STATE_RESET      0
// #define GDX_BLE_STATE_SCAN_IDLE  1
// #define GDX_BLE_STATE_SCAN_WEAK  2
// #define GDX_BLE_STATE_SCAN_FLUSH 3
// #define GDX_BLE_STATE_CONNECT    4
// #define GDX_BLE_STATE_DISCOVER   5
// #define GDX_BLE_STATE_STATUS     6
// #define GDX_BLE_STATE_SETUP      7
// #define GDX_BLE_STATE_READY      8
// #define GDX_BLE_STATE_MEASURE    9
// #define GDX_BLE_STATE_IDLE       10
// #define GDX_BLE_STATE_ERROR      11

// static int                                         g_State = GDX_BLE_STATE_RESET;
static BLEDevice                                   g_peripheral;
static BLECharacteristic                           g_d2pioCommand;
static BLECharacteristic                           g_d2pioResponse;
static struct D2PIOGetStatusCmdResponsePayload     g_status;
//static struct D2PIOGetDeviceInfoCmdResponse        g_deviceInfo;
static struct D2PIOGetSensorChannelInfoCmdResponse g_channelInfo;
static char*                                       g_deviceName;
static unsigned long                               g_sensorMask;
//static unsigned long                               g_samplePeriodInMilliseconds;
//static bool                                        g_autoConnect;
static byte                                        g_rollingCounter = 0;
static byte                                        g_ReadBuffer[256];
//static unsigned long                               g_MeasurementCounter;
static float                                       g_measurement1;
static float                                       g_measurement2;
static float                                       g_measurement3;
static float                                       g_measurement4;
static float                                       g_measurement5;
static float                                       g_measurement6;
static float                                       g_measurement7;
static int                                         g_RSSIStrength;
//static unsigned long                               g_RSSIAge;
//static byte                                        g_buffer[256];
static byte                                        g_firstEnabledSensor = 0;
static byte                                        g_secondEnabledSensor = 0;
static byte                                        g_thirdEnabledSensor = 0;
static byte                                        g_fourthEnabledSensor = 0;
static byte                                        g_fifthEnabledSensor = 0;
static byte                                        g_sixthEnabledSensor = 0;
static byte                                        g_seventhEnabledSensor = 0;
static char                                        g_firstUnits[16];
static char                                        g_firstChannelName[32];
static char                                        g_secondUnits[16];
static char                                        g_secondChannelName[32];
static char                                        g_thirdUnits[16];
static char                                        g_thirdChannelName[32];
static char                                        g_fourthUnits[16];
static char                                        g_fourthChannelName[32];
static char                                        g_fifthUnits[16];
static char                                        g_fifthChannelName[32];
static char                                        g_sixthUnits[16];
static char                                        g_sixthChannelName[32];
static char                                        g_seventhUnits[16];
static char                                        g_seventhChannelName[32];

/*
//=============================================================================
// DumpGatttService() Function
//=============================================================================
bool GDXLib::DumpGatttService(BLEDevice peripheral, char* uuid)
{
  int i;
  // Discover peripheral attributes
  delay(2000);
  //Serial.println("***Discovering service attributes ...");
  if (!peripheral.discoverService(uuid))
  {
    Serial.println("***Service attribute discovery failed!");
    return false;
  }

  int totalServices = peripheral.serviceCount();
  if (totalServices < 1) return false;

  Serial.print("**Found ");
  Serial.println(peripheral.service(uuid).uuid());
  delay(1000);

  int totalChars = peripheral.service(uuid).characteristicCount();
  #if defined DEBUG
      Serial.println("***Characteristics: ");
      for (i = 0; i < totalChars; i++)
      {
        Serial.print("***  ");
        Serial.print(peripheral.service(uuid).characteristic(i).uuid());
        Serial.println();
      }
  #endif
  return true;
}
*/

//=============================================================================
// D2PIO_DiscoverService() Function
//=============================================================================
bool GDXLib::D2PIO_DiscoverService(BLEDevice peripheral)
{
  char uuidService[]  = "d91714ef-28b9-4f91-ba16-f0d9a604f112";
  char uuidCommand[]  = "f4bf14a6-c7d5-4b6d-8aa8-df1a7c83adcb";
  char uuidResponse[] = "b41e6675-a329-40e0-aa01-44d2f444babe";

  // --------------------------------------------
  // Discover the D2PIO service
  // --------------------------------------------
  //Serial.println("***Discovering D2PIO service attributes ...");
  if (!peripheral.discoverService(uuidService))
  {
    //Serial.println("***ERROR: D2PIO service attribute discovery failed!");
    return false;
  }
  //Serial.print("***Found D2PIO service ");
  //Serial.println(peripheral.service(uuidService).uuid());

  // --------------------------------------------
  // Discover the command characteristic
  // --------------------------------------------
  g_d2pioCommand = peripheral.service(uuidService).characteristic(uuidCommand);
  if (!g_d2pioCommand)
  {
    //Serial.println("***ERROR: D2PIO command characteristic discovery failed!");
    return false;
  }
  //Serial.print("***Found D2PIO command characteristic ");
  //Serial.println(peripheral.service(uuidService).characteristic(uuidCommand).uuid());

  // --------------------------------------------
  // Discover the response characteristic
  // --------------------------------------------
  g_d2pioResponse = peripheral.service(uuidService).characteristic(uuidResponse);
  if (!g_d2pioResponse)
  {
    //Serial.println("***ERROR: D2PIO response characteristic discovery failed!");
    return false;
  }
  //Serial.print("***Found D2PIO response characteristic ");
  //Serial.println(peripheral.service(uuidService).characteristic(uuidResponse).uuid());

  if (!g_d2pioResponse.subscribe()) {
    //Serial.println("***ERROR: Failed to subscribe to D2PIO esponse characteristic");
    return false;
  }
  //Serial.println("***Subscribed to D2PIO response notifications");
  //d2pioResponse.setEventHandler(BLEValueUpdated, D2PIO_ResponseHandler);

    return true;
}

//=============================================================================
// D2PIO_CalculateChecksum() Function
//=============================================================================
byte GDXLib::D2PIO_CalculateChecksum(const byte buffer[])
{
  byte length   =  buffer[1];
  byte checksum = -buffer[3];
  byte i;

  for (i = 0; i < length; i++) checksum += buffer[i];
  return checksum;
}

//=============================================================================
// D2PIO_Dump() Function
//=============================================================================
#ifdef DEBUG
void GDXLib::D2PIO_Dump(const char* strPrefix, const byte buffer[])
{
  byte i;
  //Serial.print(strPrefix);

  for (i = 0; i < buffer[1]; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print("** ");
  }
  //Serial.println();
}
#else
#define D2PIO_Dump(strPrefix, buffer)
#endif

//=============================================================================
// D2PIO_Write() Function
//=============================================================================
bool GDXLib::D2PIO_Write(const byte buffer[])
{
  D2PIO_Dump("D2PIO >> ", buffer);

  byte lengthRemaining = buffer[1];
  byte lengthChunk;
  byte offset = 0;

  while (lengthRemaining > 0)
  {
    lengthChunk = lengthRemaining;
    //if (lengthChunk > 20) lengthChunk = 20;

    if (!g_d2pioCommand.writeValue(&buffer[offset], lengthChunk))
    {
      //Serial.println("***ERROR: D2PIO_Init write failed");
      return false;
    }

    lengthRemaining = lengthRemaining - lengthChunk;
    offset = offset + lengthChunk;
  }
  return true;
}

//=============================================================================
// D2PIO_ReadBlocking() Function
//=============================================================================
bool GDXLib::D2PIO_ReadBlocking(byte buffer[], int timeout)
{
  byte offset = 0;
  int timeoutCounter = 0;

  while (true)
  {
    // Wait for the next chunk
    while (!g_d2pioResponse.valueUpdated())
    {
      timeoutCounter++;
      if (timeoutCounter > timeout)
      {
        //Serial.println("***ERROR: D2PIO_ReadBlocking timeout!");
        return false;
      }
      delay(1);
    }

    // Copy the current chunk into the output buffer
    memcpy(&buffer[offset], g_d2pioResponse.value(), g_d2pioResponse.valueLength());
    offset = offset + g_d2pioResponse.valueLength();

    // Check if we have received the complete packet
    if ((offset >= 1) && (offset == buffer[1])) break;
  }

  D2PIO_Dump("D2PIO << ", buffer);
  return true;
}

//=============================================================================
/* GDX_ReadMeasurement() Function
    Get the measurement packet and put it in the buffer.
    Pull the measurement from the buffer according to the measurement type.
    This function does not return data, instead it stores each sensor's data 
    in the global variables g_measurement1, g_measurement2, etc..
    
    Note that this code is incomplete. For fast data collection, the packet size grows 
    in order to keep up with data collection. So a packet may contain multiple data
    points for each sensor. This code only captures the first of those packets. The
    other data points are not being captured. In addition, only testing the
    TYPE_NORMAL_REAL32
*/
//=============================================================================
bool GDXLib::GDX_ReadMeasurement(byte buffer[], int timeout)
{
  
  byte offset = 0;
  int timeoutCounter = 0;
  // Return immediately if there is nothing to do.
  while (!g_d2pioResponse.valueUpdated()){
    delay(5);//!!!may not be necessary
  }
  while (true)
  {
    // Copy the current chunk into the output buffer
    #if defined DEBUG
     Serial.print("*");
     #endif 
    memcpy(&buffer[offset], g_d2pioResponse.value(), g_d2pioResponse.valueLength());
    offset = offset + g_d2pioResponse.valueLength();
    #if defined DEBUG
        Serial.print("buffer: ");
        for (int i = 0; i < buffer[1]; i++)
          {
            Serial.print(buffer[i], HEX);
            Serial.print("** ");
          }
          Serial.println("end of buffer");
    #endif
    // Check if we have received the complete packet
    #if defined DEBUG
      Serial.println("complete packet received");
    #endif 
    // Now that we have started to receive a measurement, we must wait for all of it to arrive.
    if ((offset >= 1) && (offset == buffer[1])){
       break;
    }
  }// end of while(true)

  D2PIO_Dump("D2PIO << ", buffer);

  // Extract normal measurement packets -- NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL_REAL32
  // We only take the first measurement from the packet.  The protocol allows
  // multiple to get stuffed into one packet but we just ignore the extras.
  if (buffer[4] == NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL_REAL32)
  {
    float record;
    memcpy(&record, &buffer[9], 4);
    g_measurement1 = record;
    // Serial.print("g measurement1: ");
    // Serial.println(g_measurement1);
    // Serial.println("");

    if (g_secondEnabledSensor != 0) {
      float record2;
      memcpy(&record2, &buffer[13], 4);
      g_measurement2 = record2;
      }

    if (g_thirdEnabledSensor != 0) {
      float record3;
      memcpy(&record3, &buffer[17], 4);
      g_measurement3 = record3;
      }

    if (g_fourthEnabledSensor != 0) {
      float record4;
      memcpy(&record4, &buffer[21], 4);
      g_measurement4 = record4;
      }

    if (g_fifthEnabledSensor != 0) {
      float record5;
      memcpy(&record5, &buffer[25], 4);
      g_measurement5 = record5;
      }

    if (g_sixthEnabledSensor != 0) {
      float record6;
      memcpy(&record6, &buffer[29], 4);
      g_measurement6 = record6;
      }

    if (g_seventhEnabledSensor != 0) {
      float record7;
      memcpy(&record7, &buffer[33], 4);
      g_measurement7 = record7;
      }

    #if defined DEBUG
      Serial.print("***measurement in readMeasurement: ");
      Serial.println(measurement);
    #endif
  }
  else if (buffer[4] == NGI_BLOB_MEAS_BLOB_SUB_TYPE_WIDE_REAL32)
  {
    float record;
    memcpy(&record, &buffer[11], 4);
    g_measurement1 = record;
  }
  else if (buffer[4] == NGI_BLOB_MEAS_BLOB_SUB_TYPE_SINGLE_CHANNEL_INT32)
  {
    int32_t recordI32 = 0;
    memcpy(&recordI32, &buffer[8], 4);
    g_measurement1 = recordI32;
  }
  else if (buffer[4] == NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_INT32)
  {
    int32_t recordI32 = 0;
    memcpy(&recordI32, &buffer[8], 4);
    g_measurement1 = recordI32;
  }
  else
  {
    // Other BLOB sub-types not currently supported
    return false;
  }

  return true;
}

//=============================================================================
// D2PIO_Init() Function
//=============================================================================
bool GDXLib::D2PIO_Init()
{
  byte command[] = {
    0x58, 0x00, 0x00, 0x00, 0x1A,
    0xa5, 0x4a, 0x06, 0x49,
    0x07, 0x48, 0x08, 0x47,
    0x09, 0x46, 0x0a, 0x45,
    0x0b, 0x44, 0x0c, 0x43,
    0x0d, 0x42, 0x0e, 0x41
  };

  // Reset the rolling packet counter
  g_rollingCounter = 0xFF;

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;
  return true;
}

//=============================================================================
// D2PIO_SetMeasurementPeriod() Function
//=============================================================================
bool GDXLib::D2PIO_SetMeasurementPeriod(unsigned long measurementPeriodInMilliseconds)
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_SetMeasurmentPeriod() Function");
   #endif
   byte command[] = {
    0x58, 0x00, 0x00, 0x00, 0x1B,
    0xFF, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };

  // Convert to milliseconds and populate the payload
  unsigned long measurementPeriodInMicroseconds = measurementPeriodInMilliseconds * 1000;
  command[7]  = (measurementPeriodInMicroseconds >> 0)  & 0xFF;
  command[8]  = (measurementPeriodInMicroseconds >> 8)  & 0xFF;
  command[9]  = (measurementPeriodInMicroseconds >> 16) & 0xFF;
  command[10] = (measurementPeriodInMicroseconds >> 24) & 0xFF;

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;
  return true;
}

//=============================================================================
// D2PIO_GetAvailableChannels() Function
//=============================================================================
bool GDXLib::D2PIO_GetAvailableChannels(unsigned long& availableMask)
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_GetAvailableChannels() Function");
   #endif
  
  byte command[] = {
  0x58, 0x00, 0x00, 0x00, 0x51
  };

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;

  // Extract the channel mask from the packet
  unsigned long mask;
  memcpy(&mask, &g_ReadBuffer[6], 4);
  availableMask = mask;
  return true;
}
//=============================================================================
// D2PIO_GetDefaultChannels() Function
//=============================================================================
bool GDXLib::D2PIO_GetDefaultChannels(unsigned long& defaultMask)
{
    #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_GetDefaultChannels() Function");
    #endif
    byte command[] = {
    0x58, 0x00, 0x00, 0x00, 0x56
  };

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;

  // Extract the channel mask from the packet
  unsigned long mask;
  memcpy(&mask, &g_ReadBuffer[6], 4); 
  defaultMask = mask;
  return true;
}
//=============================================================================
// D2PIO_GetStatus() Function
//=============================================================================
bool GDXLib::D2PIO_GetStatus()
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_GetStatus() Function");
   #endif
  byte command[] = {
    0x58, 0x00, 0x00, 0x00, D2PIO_CMD_ID_GET_STATUS
  };

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;

  D2PIOGetStatusCmdResponsePayload* pResponse;
  pResponse = (D2PIOGetStatusCmdResponsePayload*)&g_ReadBuffer[6];
  memcpy(&g_status, pResponse, sizeof(D2PIOGetStatusCmdResponsePayload));
  #if defined DEBUG
    Serial.println("***Device status:");
    Serial.print("***  Status: ");
    Serial.println(pResponse->status);
  #endif
  chargerStatus= (pResponse->chargerState);
  return true;
}

//=============================================================================
// D2PIO_GetChannelInfo() Function
//=============================================================================
// Get the info for the specific channel and store it in g_ChannelInfo. Get the units
// and channel name from getUnits() and getSensorName()

void GDXLib::D2PIO_GetChannelInfo(byte channelNumber)
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_GetChannelInfo() Function");
   #endif
   byte command[] = {
    0x58, 0x00, 0x00, 0x00, D2PIO_CMD_ID_GET_SENSOR_CHANNEL_INFO,
    0x00
  };

  // Specify the channel number parameter
  command[5] = channelNumber;
  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  D2PIO_Write(command);
  D2PIO_ReadBlocking(g_ReadBuffer, 5000);
  D2PIOGetSensorChannelInfoCmdResponse* pResponse;
  pResponse = (D2PIOGetSensorChannelInfoCmdResponse*)&g_ReadBuffer[6];
  memcpy(&g_channelInfo, pResponse, sizeof(D2PIOGetSensorChannelInfoCmdResponse));
  #if defined DEBUG   
        Serial.print("***Channel[");
        Serial.println(channelNumber);
        Serial.println("***] info:");
        Serial.print("***  Description: ");
        Serial.println(pResponse->sensorDescription); 
        Serial.print("***  ID: ");
        Serial.println(pResponse->sensorId);
        Serial.print("***  Measurement type: ");
        Serial.println(pResponse->numericMeasType);
        Serial.print("***  Sampling mode: ");
        Serial.println(pResponse->samplingMode);
        Serial.print("***  Units: ");
        Serial.println(pResponse->sensorUnit);
        Serial.print("***  Measurement uncertainty: ");
        Serial.println(pResponse->measurementUncertainty);
        Serial.print("***  Measurement min: ");
        Serial.println(pResponse->minMeasurement);
        Serial.print("***  Measurement max: ");
        Serial.println(pResponse->maxMeasurement);
        Serial.print("***  Period typical: ");
        Serial.println(pResponse->typMeasurementPeriod);
        Serial.print("***  Period min: ");
        Serial.println(pResponse->minMeasurementPeriod);
        Serial.print("***  Period max: ");
        Serial.println((long int)(pResponse->maxMeasurementPeriod));
        Serial.print("***  Period granularity: ");
        Serial.println(pResponse->measurementPeriodGranularity);
        Serial.print("***  Mutual exclusion mask: 0x");
        Serial.println(pResponse->mutualExclusionMask);
  #endif
}

//=============================================================================
// D2PIO_GetChannelInfoAll() Function
//=============================================================================
bool GDXLib::D2PIO_GetChannelInfoAll()
{
   #if defined DEBUG
      Serial.println ("*** in D2PIO_GetChannelInfoAll() Function");
   #endif
   unsigned long availableMask = 0;
   unsigned long testMask = 1;
   byte i;

   if (!D2PIO_GetAvailableChannels(availableMask)) return false;

   for (i = 0; i < 32; i++)
   {
    if (testMask & availableMask)
    {
      //if (!D2PIO_GetChannelInfo(i)) return false;
      D2PIO_GetChannelInfo(i);
    }
    testMask = testMask << 1;
   }
   return true;
}

//=============================================================================
// GDX_getDefaultSensor() Function
//=============================================================================
byte GDXLib::GDX_getDefaultSensor()
{
  unsigned long availableMask = 0;
  unsigned long defaultMask = 0;
  unsigned long testMask = 1;
  byte i;
  
  D2PIO_GetAvailableChannels(availableMask);
  D2PIO_GetDefaultChannels(defaultMask);
  // Select the first channel number that is called out in the default mask
  for (i = 0; i < 32; i++)
  {
    if (testMask & defaultMask & availableMask) return i;//use the default channel
    testMask = testMask << 1;
  }
  if (i == 32) return 1; //if this did not work, just return 1 as the default
}

//=============================================================================
// GDX_StartMeasurements() Function
//=============================================================================
bool GDXLib::GDX_StartMeasurements(unsigned long sensorMask)
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_StartMeasurement() Function");
   #endif
  //  Serial.print("GDX_StartMeasurement sensormask: ");
  //  Serial.println(sensorMask);
   byte command[] = {
    0x58, 0x00, 0x00, 0x00, 0x18,
    0xFF,
    0x01,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

  command[7]  = (sensorMask >> 0)  & 0xFF;
  command[8]  = (sensorMask >> 8)  & 0xFF;
  command[9]  = (sensorMask >> 16) & 0xFF;
  command[10] = (sensorMask >> 24) & 0xFF;

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;  
  return true;
}

//=============================================================================
// GDX_StopMeasurements() Function
//=============================================================================
bool GDXLib::GDX_StopMeasurements()
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_StopMeasurement() Function");
   #endif

   byte command[] = {
    0x58, 0x00, 0x00, 0x00, 0x19,
			0xFF,
			0x00,
			0xFF, 0xFF, 0xFF, 0xFF
};

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;  
  return true;
}

//=============================================================================
/* open(char* deviceName) Function
    Call the ArduinoBLE BLE.begin() function to initialize the Arduino boards' BLE.
    Then scan for either a specific Go Direct device or the nearest.
    The deviceName argument must be the full name and serial number 
    in quotation marks(i.e, "GDX-HD 151000C1"), or "proximity" for opening
    the nearest device (with a threshold that is stronger than -60).
*/
//=============================================================================
bool GDXLib::open(char* deviceName)
{
  g_deviceName = deviceName;
  
  #if defined DEBUG
    Serial.println("***in open()");
    Serial.print("*** searching for "); 
    Serial.println(deviceName);
  #endif

  if (!BLE.begin()) {
      // Serial.println("starting Bluetooth® Low Energy module failed");
      // Serial.println("Disconnect, and then reconnect, the Arduino USB cable");
      return false;
  }

  if (g_deviceName == "proximity") {
    // Serial.println("in proximity case..");
    if (!GoDirectBLE_Scan_Proximity())
      return false;
  }
  else {
    // Serial.println("in scan for name case..");
    if (!GoDirectBLE_Scan_For_Name())
      return false;
  }

  if (!GoDirectBLE_Connect())
    return false;

  if (!GoDirectBLE_Discover_Attributes())
    return false;

  if (!D2PIO_DiscoverService(g_peripheral))
    return false;

  if (!D2PIO_Init())
    return false;

  // Wait for connection interval to finish negotiating
  delay(1000);

  if (!D2PIO_GetStatus())
    return false;

  if (!D2PIO_GetChannelInfoAll())
    return false;
  

  _RSSI=GoDirectBLE_GetScanRSSI(); 
  _batteryPercent=GoDirectBLE_GetBatteryStatus();
  _chargeState   =GoDirectBLE_GetChargeStatus();
  sprintf(_orderCode,"%s",GoDirectBLE_GetOrderCode());
  sprintf(_serialNumber,"%s",GoDirectBLE_GetSerialNumber());

  #if defined DEBUG
  Serial.println("***HERE is all the info");
  Serial.print("*** _RSSI"); 
  Serial.println(_RSSI);
  Serial.print("*** _batteryPercent");
  Serial.println(_batteryPercent);
  Serial.print("***_chargeState");
  Serial.println(_chargeState);
  #endif
  return true;
} 

//=============================================================================
// GoDirectBLE_Scan_Proximity() Function
//=============================================================================
  bool GDXLib::GoDirectBLE_Scan_Proximity()
  {

    // Serial.print("Begin proximity scan for nearest Go Direct");
    // Serial.println();

    BLE.scan(false); //
    delay(100);
    String strongest_device = "None";
    String final_device = "None";
    int strongest_rssi = -1000;
    int threshold = -60; //modify threshold if needed
    int i = 0;

    // loop until a peripheral is found
    while (true) {

      // check if a peripheral has been discovered
      BLEDevice peripheral = BLE.available();
      // Serial.print(" i = ");
      // Serial.println(i);

      if (peripheral) {
        // Serial.println("Discovered a peripheral");
        // Serial.print("Local Name: ");
        
        if (peripheral.hasLocalName()) {
          // Serial.println(peripheral.localName());
          // Serial.println("Is this GDX:  ");
        
          if ((peripheral.localName()[0] == 'G') &&
          (peripheral.localName()[1] == 'D') &&
          (peripheral.localName()[2] == 'X')) {
          
            // Serial.println("YES");
            // Serial.print("RSSI: ");
            // Serial.println(peripheral.rssi());

            if (peripheral.rssi() > strongest_rssi) {
              strongest_rssi = peripheral.rssi();
              strongest_device = peripheral.localName();
              // Serial.print("Set strongest RSSI to: ");
              // Serial.println(peripheral.localName());
              // Serial.println("");
              g_peripheral = peripheral;
            }
          }
          // else {
          //   Serial.println("NO");
          // }
        }
      }
      // else {
      //   Serial.println("no peripheral found");
      // }
      if (i > 10) {
        if (strongest_rssi > threshold) {
          //Serial.println("Discovered proximity device! Scan stopped");
          BLE.stopScan();
          break; //device has been found
        }
        // else {
        //   Serial.println("no devices with rssi lower than threshold");
        // }
      }
      delay(100);
      i ++;
    }  
  return true; //device was found, while loop exited, end Scan For Name
  }
      
//=============================================================================
// GoDirectBLE_Scan_For_Name() Function
//=============================================================================
  bool GDXLib::GoDirectBLE_Scan_For_Name()
  {
    // Serial.print("Begin BLE Scan for name: ");
    // Serial.println(g_deviceName);
    // Serial.println();

    BLE.scanForName(g_deviceName);
    delay(1000);

    // loop until a peripheral is found
    while (true) {
      BLEDevice peripheral = BLE.available();
      if (peripheral) {     //escape while loop if found
        // discovered a peripheral
        //Serial.println("Discovered the Go Direct peripheral! Scan stopped");
        BLE.stopScan();
        g_peripheral = peripheral;
        break;
      }
    delay(1000);
    //Serial.println(" No peripheral, Scan again");
    }
    return true;
  } //end Scan

//=============================================================================
// GoDirectBLE_Connect() Function
//=============================================================================
  bool GDXLib::GoDirectBLE_Connect()
  {  
    if (!g_peripheral.connect()) {
      // Serial.println("Failed to connect!");
      // Serial.println("Disconnect USB cable, reconnect, and run the Upload again");
      return false;
    }
    // Serial.println("Connected");
    // Serial.println();
    return true;
  }

//=============================================================================
// GoDirectBLE_Discover_Attributes() Function
//=============================================================================
  bool GDXLib::GoDirectBLE_Discover_Attributes()
  {    
    // discover peripheral attributes
    //Serial.println("Discovering attributes ...");
    if (!g_peripheral.discoverAttributes()) {
      // Serial.println("Attribute discovery failed!");
      // Serial.println("Disconnect USB cable, reconnect, and run the Upload again");
      g_peripheral.disconnect();
      //BLE.end(); Jorge only suggested peripheral.disonnect(), not BLE.end()
      return false;
    }
    
    //Serial.println("Attributes discovered");
    return true;
  }

//=============================================================================
/* enableSensor(byte selectedSensor) Function
    This function allows the user to select a specific sensor for data 
    collection.
    
    Each Go Direct device has one or more onboard sensors. These sensors have a 
    unique sensor number. Use the sensor number to enable the sensor. Use this same
    sensor number for all of the other functions that require a selectedSensor argument. 
    
    This function takes the selectedSensor number and stores it in a 
    global variable that marks it as enabled. For example, if the user wants to 
    collect data from sensor #4 only, this will get stored in the g_firstEnabledSensor
    global variable. If they want to collect from sensor 4 and 5, then 
    g_firstEnabledSensor = 4, and g_secondEnabledSensor = 5.

    if the selectedSensor argument=255, this signifies using the default sensor. Use 
    the 255 value in all of the other functions that require a selectedSensor argument.
    
    The units and name for the selected sensor are stored in
    global variables. See the functions getUnits() and getSensorName(). In addition,
    the sensor mask is calculated and stored in a global variable to be used in start()

*/
//=============================================================================!@
   void GDXLib::enableSensor(byte selectedSensor) {

    if (selectedSensor == 255) {
      selectedSensor = GDX_getDefaultSensor();
      // Serial.print("default sensor = ");
      // Serial.println(selectedSensor);
    }

    // if firstEnabled does not yet have a sensor number assigned, store it here.
    if (g_firstEnabledSensor == 0) {
      g_firstEnabledSensor = selectedSensor;
      char* firstUnits;
      char* firstName;
      D2PIO_GetChannelInfo(selectedSensor); //this function stores the ch info in g_channelInfo
      firstUnits = g_channelInfo.sensorUnit;
      firstName = g_channelInfo.sensorDescription;
      strcpy(g_firstUnits, firstUnits);
      strcpy(g_firstChannelName, firstName);
    }

    // if secondEnabled does not yet have a sensor assigned, assign it selectedSensor
    else if (g_secondEnabledSensor == 0) {
      g_secondEnabledSensor = selectedSensor;
      char* secondUnits;
      char* secondName;
      D2PIO_GetChannelInfo(selectedSensor);
      secondUnits = g_channelInfo.sensorUnit;
      secondName = g_channelInfo.sensorDescription;
      strcpy(g_secondUnits, secondUnits);
      strcpy(g_secondChannelName, secondName);
    }

    else if (g_thirdEnabledSensor = selectedSensor) {
      g_thirdEnabledSensor = selectedSensor;
      char* thirdUnits;
      char* thirdName;
      D2PIO_GetChannelInfo(selectedSensor);
      thirdUnits = g_channelInfo.sensorUnit;
      thirdName = g_channelInfo.sensorDescription;
      strcpy(g_thirdUnits, thirdUnits);
      strcpy(g_thirdChannelName, thirdName);
    }

    else if (g_fourthEnabledSensor = selectedSensor) {
      g_fourthEnabledSensor = selectedSensor;
      char* fourthUnits;
      char* fourthName;
      D2PIO_GetChannelInfo(selectedSensor);
      fourthUnits = g_channelInfo.sensorUnit;
      fourthName = g_channelInfo.sensorDescription;
      strcpy(g_fourthUnits, fourthUnits);
      strcpy(g_fourthChannelName, fourthName);
    }

    else if (g_fifthEnabledSensor = selectedSensor) {
      g_fifthEnabledSensor = selectedSensor;
      char* fifthUnits;
      char* fifthName;
      D2PIO_GetChannelInfo(selectedSensor);
      fifthUnits = g_channelInfo.sensorUnit;
      fifthName = g_channelInfo.sensorDescription;
      strcpy(g_fifthUnits, fifthUnits);
      strcpy(g_fifthChannelName, fifthName);
    }

    else if (g_sixthEnabledSensor = selectedSensor) {
      g_sixthEnabledSensor = selectedSensor;
      char* sixthUnits;
      char* sixthName;
      D2PIO_GetChannelInfo(selectedSensor);
      sixthUnits = g_channelInfo.sensorUnit;
      sixthName = g_channelInfo.sensorDescription;
      strcpy(g_sixthUnits, sixthUnits);
      strcpy(g_sixthChannelName, sixthName);
    }

    else {
      g_seventhEnabledSensor = selectedSensor;
      char* seventhUnits;
      char* seventhName;
      D2PIO_GetChannelInfo(selectedSensor);
      seventhUnits = g_channelInfo.sensorUnit;
      seventhName = g_channelInfo.sensorDescription;
      strcpy(g_seventhUnits, seventhUnits);
      strcpy(g_seventhChannelName, seventhName);
    }

    // Convert the channel numbers to a mask and store in a global variable
    // to be used in the start() function.
    unsigned long sensorMask = 0;

    if (g_firstEnabledSensor != 0) sensorMask += (1 << g_firstEnabledSensor);
    if (g_secondEnabledSensor != 0) sensorMask += (1 << g_secondEnabledSensor);
    if (g_thirdEnabledSensor != 0) sensorMask += (1 << g_thirdEnabledSensor);
    if (g_fourthEnabledSensor != 0) sensorMask += (1 << g_fourthEnabledSensor);
    if (g_fifthEnabledSensor != 0) sensorMask += (1 << g_fifthEnabledSensor);
    if (g_sixthEnabledSensor != 0) sensorMask += (1 << g_sixthEnabledSensor);
    if (g_seventhEnabledSensor != 0) sensorMask += (1 << g_seventhEnabledSensor);
    // Serial.print("sensor mask: ");
    // Serial.println(sensorMask);
    g_sensorMask = sensorMask;
   }

 //=============================================================================
/* start(unsigned long period) Function
    Start collecting data from the sensors that were selected in the enableSensor() function.
    All of the sensor numbers set with enableSensor() are used to compute a sensor mask. This single
    value masks all of the individual values of the sensors.

    The period (time between samples) is set in milliseconds. 
    For example, start(500) has a period of 500 ms or 0.5 seconds, which is the same as 
    having a sample rate of 2 samples/second. 
*/
//=============================================================================!@
   void GDXLib::start(unsigned long period) {
    #if defined DEBUG
    Serial.print("**$ calling start function "); 
    #endif
    D2PIO_SetMeasurementPeriod(period);
    GDX_StartMeasurements(g_sensorMask); 
   }

 //=============================================================================
// stop() Function
//=============================================================================!@
   void GDXLib::stop() {
    #if defined DEBUG
    Serial.print("**$ calling stop function "); 
    #endif
    GDX_StopMeasurements();
   }

//=============================================================================
/* read() Function
    Call read at least as fast as the period. This will fill the buffer, but does
    not return data. The getMeasurement() function is used to return the specific 
    sensor's value out of the buffer.

    Note that the code drops data points if fast data collection sends data back in packets.
    There is a 5 second timeout, so do not set the period > 5000 ms
*/
//=============================================================================!@
void GDXLib::read() 
{
GDX_ReadMeasurement(g_ReadBuffer, 5000);

  }

//=============================================================================
/* getMeasurement(byte selectedSensor) Function
    The read() function gets the data and stores the measurement(s) in the g_measurement 
    global variables. This function is used to retrieve the g_measurement data
    for the specified sensor.
*/
//=============================================================================
float GDXLib::getMeasurement(byte selectedSensor)
{
  
  if (g_firstEnabledSensor == selectedSensor || selectedSensor == 255) return g_measurement1;
  else if (g_secondEnabledSensor == selectedSensor) return g_measurement2;
  else if (g_thirdEnabledSensor == selectedSensor) return g_measurement3;
  else if (g_fourthEnabledSensor == selectedSensor) return g_measurement4;
  else if (g_fifthEnabledSensor == selectedSensor) return g_measurement5;
  else if (g_sixthEnabledSensor == selectedSensor) return g_measurement6;
  else if (g_seventhEnabledSensor == selectedSensor) return g_measurement7;
  
  else return 0;
  
}

//=============================================================================
/* getUnits(byte selectedSensor) Function
    The enableSensor() function uses the GetChannelInfo function to get the selected
    sensor's units and name and store them in global variables (e.g., g_firstUnits, g_firstName). 
    This function is used to retrieve the g_firstUnits (or g_secondUnits if there are 
    two sensors enabled) value for the specified sensor.
*/
//=============================================================================
const char* GDXLib::getUnits(byte selectedSensor)
{
  if (g_firstEnabledSensor == selectedSensor || selectedSensor == 255) return g_firstUnits;
  else if (g_secondEnabledSensor == selectedSensor) return g_secondUnits;
  else if (g_thirdEnabledSensor == selectedSensor) return g_thirdUnits;
  else if (g_fourthEnabledSensor == selectedSensor) return g_fourthUnits;
  else if (g_fifthEnabledSensor == selectedSensor) return g_fifthUnits;
  else if (g_sixthEnabledSensor == selectedSensor) return g_sixthUnits;
  else return g_seventhUnits;
}


//=============================================================================
/* getSensorName(byte selectedSensor) Function
    The enableSensor() function uses the GetChannelInfo function to get the selected
    sensor's units and name and store them in global variables (e.g., g_firstUnits, g_firstName). 
    This function is used to retrieve the g_firstName (or g_secondName if there are 
    two sensors enabled) value for the specified sensor.
*/
//=============================================================================
const char* GDXLib::getSensorName(byte selectedSensor)
{  
  if (g_firstEnabledSensor == selectedSensor) return g_firstChannelName;
  else if (g_secondEnabledSensor == selectedSensor) return g_secondChannelName;
  else if (g_thirdEnabledSensor == selectedSensor) return g_thirdChannelName;
  else if (g_fourthEnabledSensor == selectedSensor) return g_fourthChannelName;
  else if (g_fifthEnabledSensor == selectedSensor) return g_fifthChannelName;
  else if (g_sixthEnabledSensor == selectedSensor) return g_sixthChannelName;
  else  return g_seventhChannelName;
}

//=============================================================================
// getDeviceName() Function
//=============================================================================
const char* GDXLib::getDeviceName()
// 

{  
  static char strBuffer[32];
  strcpy(strBuffer, g_peripheral.localName().c_str());
  const char* pch = strtok(strBuffer, NULL);
  return pch;
}

//=============================================================================
// GoDirectBLE_GetSerialNumber() Function
//=============================================================================
const char* GDXLib::GoDirectBLE_GetSerialNumber()
{
  static char strBuffer[32];
  strcpy(strBuffer, g_peripheral.localName().c_str());
  const char* pch = strtok(strBuffer, " ");
  pch = strtok (NULL, " ");
  return pch;
}

//=============================================================================
// GoDirectBLE_GetOrderCode() Function
//=============================================================================
const char* GDXLib::GoDirectBLE_GetOrderCode()
{
  static char strBuffer[32];
  strcpy(strBuffer, g_peripheral.localName().c_str());
  const char* pch = strtok(strBuffer, " ");
  return pch;
}


//=============================================================================
// GoDirectBLE_GetBatteryStatus() Function
//=============================================================================
uint8_t GDXLib::GoDirectBLE_GetBatteryStatus()
{
  return g_status.batteryLevelPercent;
}
//=============================================================================
// GoDirectBLE_GetChargeStatus() Function
//=============================================================================
uint8_t GDXLib::GoDirectBLE_GetChargeStatus()
{
  return g_status.chargerState;
}
//=============================================================================
// GoDirectBLE_GetScanRSSI() Function 
//=============================================================================
int GDXLib::GoDirectBLE_GetScanRSSI()
{
  return g_RSSIStrength;
}
//=============================================================================
// GoDirectBLE_DisplayChannelAsInteger() Function I do not think we use this, but until I deal with everything digital, I am leaving it in.!!!
//=============================================================================
bool GDXLib::GoDirectBLE_DisplayChannelAsInteger()
{
  return (g_channelInfo.numericMeasType == 1);
}

//=============================================================================
// GoDirectBLE_End() Function 
//=============================================================================
void GDXLib::close()
{
  //BLE.disconnect(); Jorge only suggested peripheral.disconnect()
  g_peripheral.disconnect();
  BLE.end();  //jorge did not suggest this, but is it more reliable?
  #if defined DEBUG
     Serial.println("*** BlE connection closed");
  #endif
}
