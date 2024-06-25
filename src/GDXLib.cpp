/* This is a library to make using Vernier GDX sensors 
 easy using an Arduino which supports the Arduino BLE library
 
Version 0.90 - addition of support for begin(GDX*ACC XXXXXXXXX) and minor clean up
Also, I changed the word "Begin" to "open"
and it supports samplePeriodInMilliseconds.
---  
*/
//#define DEBUG //NOTE THIS PRINTS OUT DECODING STUFF!!!
#include "ArduinoBLE.h"
#include "Arduino.h"
#include "GDXLib.h"
#define GDXLib_LIB_VERSION "0.88"//automatically displayed

GDXLib::GDXLib()
{}
char channelName[32];
char deviceName[32];
char channelUnits[16];
int channelNumber;
uint8_t chargerStatus;
int batteryPercent;
//char strBuffer[32];
char strFW1[16];// These are not used in my code
char strFW2[16];
#define D2PIO_CMD_ID_GET_STATUS                          0x10
#define D2PIO_CMD_ID_START_MEASUREMENTS                  0x18
#define D2PIO_CMD_ID_STOP_MEASUREMENTS                   0x19
#define D2PIO_CMD_ID_INIT                                0x1A
#define D2PIO_CMD_ID_SET_MEASUREMENT_PERIOD              0x1B
#define D2PIO_CMD_ID_CLEAR_ERROR_FLAGS                   0x34
#define D2PIO_CMD_ID_GET_SENSOR_CHANNEL_INFO             0x50
#define D2PIO_CMD_ID_GET_SENSOR_CHANNELS_AVAILABLE_MASK  0x51
#define D2PIO_CMD_ID_DISCONNECT                          0x54
#define D2PIO_CMD_ID_GET_DEVICE_INFO                     0x55
#define D2PIO_CMD_ID_GET_SENSOR_CHANNELS_DEFAULT_MASK    0x56
#define D2PIO_CMD_ID_IDENTIFY                            0x58
#define D2PIO_CMD_ID_GET_BATTERY_LEVEL                   0x78

#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL                   0
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_DIGITAL        1
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_ANALOG         2
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_MULTIPLE_PERIODS         3
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_PERIODIC_DIGITAL_COUNTS  4
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL_SIGNED            5
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NORMAL_REAL32            6
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_WIDE_REAL32              7
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_SINGLE_CHANNEL_REAL32    8
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_SINGLE_CHANNEL_INT32     9
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_REAL32         10
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_INT32          11
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_NEXT_PERIODIC_TIMESTAMP  12
#define NGI_BLOB_MEAS_BLOB_SUB_TYPE_DROPPED_MEASUREMENT      13

#define D2PIO_MASK_STATUS_ERROR_CMD_NOT_RECOGNIZED      0x01
#define D2PIO_MASK_STATUS_ERROR_CMD_IGNORED             0x02
#define D2PIO_MASK_STATUS_ADC_UNCALIBRATED              0x04
#define D2PIO_MASK_STATUS_AUTOID_UNCALIBRATED           0x08
#define D2PIO_MASK_STATUS_ERROR_INTERNAL_ERROR1         0x10
#define D2PIO_MASK_STATUS_ERROR_AUDIO_CONTROL_FAILURE   0x10
#define D2PIO_MASK_STATUS_ERROR_INTERNAL_ERROR2         0x20
#define D2PIO_MASK_STATUS_ERROR_AUDIO_STREAM_FAILURE    0x20
#define D2PIO_MASK_STATUS_ERROR_MASTER_FIFO_OVERFLOW    0x40
#define D2PIO_MASK_STATUS_ERROR_DIGITAL_TRANSITION_LOST 0x80

#define D2PIO_CHARGER_STATE_IDLE     0
#define D2PIO_CHARGER_STATE_CHARGING 1
#define D2PIO_CHARGER_STATE_COMPLETE 2
#define D2PIO_CHARGER_STATE_ERROR    3

#define D2PIO_MAX_ORDERCODE_LENGTH 16
#define D2PIO_MAX_SERIALNUMBER_LENGTH 16
#define D2PIO_BLE_ADDRESS_LENGTH 6
#define D2PIO_MAX_DEVICENAME_LENGTH 32
#define D2PIO_MAX_DEVICEDESCRIPTION_LENGTH 64
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

struct D2PIOStartMeasurementsParams
{
  int8_t   channel;  /* -1 => all channels specified by channelBits. */
  uint8_t  sampleAtTime0flag;
  uint32_t channelBits;    //Id's channels that should start taking measurements. Ignored unless (channel == -1).
  int64_t  startTimestamp; //Microseconds since Jan 1 1970 Coordinated Universal Time(aka UTC aka Greenwich Mean Time), (timestamp == 0) => start immediately.
} __attribute__ ((packed));

struct D2PIOStopMeasurementsParams
{
  int8_t   channel;  /* -1 => all channels specified by channelBits. */
  uint8_t  spare;
  uint32_t channelBits;    //Id's channels that should start taking measurements. Ignored unless (channel == -1).
} __attribute__ ((packed));

struct D2PIOSetMeasurementPeriodParams
{
  int8_t   channel;  /* -1 => all channels specified by channelBits. */
  uint8_t  spare;
  uint64_t measurementPeriod; //In 'ticks', typically a tick == 1 microsecond
} __attribute__ ((packed));

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

struct D2PIOGetDeviceInfoCmdResponse
{
  char     OrderCode[D2PIO_MAX_ORDERCODE_LENGTH];//NULL terminated UTF-8 string.
  char     SerialNumber[D2PIO_MAX_SERIALNUMBER_LENGTH];//NULL terminated UTF-8 string.
  char     DeviceName[D2PIO_MAX_DEVICENAME_LENGTH];//NULL terminated UTF-8 string.
  uint16_t manufacturerId;
  uint16_t manufacturedYear;
  uint8_t  ManufacturedMonth;
  uint8_t  ManufacturedDay;
  uint8_t  majorVersionMasterCPU;
  uint8_t  minorVersionMasterCPU;
  uint16_t buildNumMasterCPU;
  uint8_t  majorVersionSlaveCPU;
  uint8_t  minorVersionSlaveCPU;
  uint16_t buildNumSlaveCPU;
  uint8_t  BLE_Address[D2PIO_BLE_ADDRESS_LENGTH];  // Raw binary, not NULL terminated string. Displays like this --> [5]:[4]:[3]:[2]:[1]:[0]
  uint32_t NVMemSize;
  char     DeviceDescription[D2PIO_MAX_DEVICEDESCRIPTION_LENGTH];//NULL terminated UTF-8 string.
} __attribute__ ((packed));

//the line below specifies the threshold RSSI for connecting!!!
#define GDX_BLE_AUTO_CONNECT_RSSI_THRESHOLD -44
#define GDX_BLE_STATE_RESET      0
#define GDX_BLE_STATE_SCAN_IDLE  1
#define GDX_BLE_STATE_SCAN_WEAK  2
#define GDX_BLE_STATE_SCAN_FLUSH 3
#define GDX_BLE_STATE_CONNECT    4
#define GDX_BLE_STATE_DISCOVER   5
#define GDX_BLE_STATE_STATUS     6
#define GDX_BLE_STATE_SETUP      7
#define GDX_BLE_STATE_READY      8
#define GDX_BLE_STATE_MEASURE    9
#define GDX_BLE_STATE_IDLE       10
#define GDX_BLE_STATE_ERROR      11

#define D2PIO_SCAN_RESULT_NONE     0
#define D2PIO_SCAN_RESULT_WEAK     1
#define D2PIO_SCAN_RESULT_FLUSH    2
#define D2PIO_SCAN_RESULT_SUCCESS  3

static int                                         g_State = GDX_BLE_STATE_RESET;
static BLEDevice                                   g_peripheral;
static BLECharacteristic                           g_d2pioCommand;
static BLECharacteristic                           g_d2pioResponse;
static struct D2PIOGetStatusCmdResponsePayload     g_status;
static struct D2PIOGetDeviceInfoCmdResponse        g_deviceInfo;
static struct D2PIOGetSensorChannelInfoCmdResponse g_channelInfo;
static char*                                       g_deviceName;
static byte                                        g_channelNumber;
static unsigned long                               g_samplePeriodInMilliseconds;
static bool                                        g_autoConnect;
static byte                                        g_rollingCounter = 0;
static byte                                        g_ReadBuffer[256];
static unsigned long                               g_MeasurementCounter;
static float                                       g_measurement;
static int                                         g_RSSIStrength;
static unsigned long                               g_RSSIAge;


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
//=============================================================================
// D2PIO_Scan() Function
//=============================================================================
int D2PIO_Scan(bool useRssiThreshold, int threshold)//useRssiThreshold is autoconnect!!!
{
  // Check if a peripheral has been discovered
  #if defined DEBUG
     Serial.print("***in D2PIO_Scan, useRssiThreshold= ");
     Serial.print(useRssiThreshold);
     Serial.print("*** threshold= ");
     Serial.println(threshold);
  #endif
  BLEDevice peripheral = BLE.available();
  if (!peripheral)
  {
    Serial.println("No peripheral");
    // Age the last known RSSI reading
    if (g_RSSIStrength > 0) g_RSSIAge++;

    // Flush the RSSI if it is too old
    if (g_RSSIAge > 200000)
    {
      g_RSSIAge = 0;
      g_RSSIStrength = 0;
      return D2PIO_SCAN_RESULT_FLUSH;
    }

    return D2PIO_SCAN_RESULT_NONE;
  }
   Serial.print (".");//'.' shows test for GDX-
  // Make sure we found a GDX device (always)
  if ((peripheral.localName()[0] != 'G') ||
      (peripheral.localName()[1] != 'D') ||
      (peripheral.localName()[2] != 'X'))
         {
           Serial.print("x");//x means not a GDX device
           return D2PIO_SCAN_RESULT_NONE;
         }
      Serial.print ("GDX ");//move to new line if sensor is GDX
  if (g_deviceName[3]=='*'){ 
       //the code below checks to see if we have the right type of GDX sensor
       //in the cases where the '*' was used in the open (only).
       Serial.println("*");//'' shows test for just the right order code
     if ((peripheral.localName()[4] != g_deviceName[4]) ||
     (peripheral.localName()[5] != g_deviceName[5]) ||
     (peripheral.localName()[6] != g_deviceName[6])){
       Serial.print("!");// indicates the wrong GDX device type
       return D2PIO_SCAN_RESULT_NONE;
     }//end of if right order code
    }//end of '*' checking
  Serial.println ("");//move to new line, when sensor is found
  // Create a relative strength reading from 0 to 16
  // 0  = Very weak
  // 16 = Strong enough to connect
  g_RSSIAge = 0;
  int rssiTestLevel = threshold;
  for (g_RSSIStrength = 16; g_RSSIStrength > 0; g_RSSIStrength--)
  {
    if (peripheral.rssi() > rssiTestLevel) break;
    rssiTestLevel = rssiTestLevel - 2;
  }
g_RSSIStrength=peripheral.rssi();
#if defined DEBUG
    Serial.print("***Found ");
    Serial.print(peripheral.localName());
    Serial.print("*** at ");
    Serial.print(peripheral.address());
    Serial.print("*** with RSSI ");//This is the value I want
    Serial.print(peripheral.rssi());
    Serial.println();
    Serial.print("***peripheral.rssi() ");
    Serial.println(peripheral.rssi());

    uint8_t mfgData[64];
    uint8_t mgfDataLen = 0;
    Serial.print("***MfgData=");
    Serial.print("*** ");
    for (int i = 0; i < mgfDataLen; i++)
      {
        Serial.print(mfgData[i], HEX);
        Serial.print("*** ");
      }
      Serial.println();
 #endif
 
  // Check RSSI threshold (optional)
  //how does useRssiThreshold get set to true !!!
  if (useRssiThreshold && (peripheral.rssi() < threshold)) 
          return D2PIO_SCAN_RESULT_WEAK;
   g_peripheral = peripheral;
  return D2PIO_SCAN_RESULT_SUCCESS;
}

//=============================================================================
// D2PIO_DiscoverService() Function
//=============================================================================
//bool GDXLib::GDXLib::D2PIO_DiscoverService(BLEDevice peripheral)
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
    Serial.println("***ERROR: D2PIO service attribute discovery failed!");
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
    Serial.println("***ERROR: D2PIO command characteristic discovery failed!");
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
    Serial.println("***ERROR: D2PIO response characteristic discovery failed!");
    return false;
  }
  //Serial.print("***Found D2PIO response characteristic ");
  //Serial.println(peripheral.service(uuidService).characteristic(uuidResponse).uuid());

  if (!g_d2pioResponse.subscribe()) {
    Serial.println("***ERROR: Failed to subscribe to D2PIO esponse characteristic");
    return false;
  }
  //Serial.println("***Subscribed to D2PIO response notifications");
  //d2pioResponse.setEventHandler(BLEValueUpdated, D2PIO_ResponseHandler);

    return true;
}

//=============================================================================
// D2PIO_CalculateChecksum() Function
//=============================================================================
//byte GDXLib::GDXLib::D2PIO_CalculateChecksum(const byte buffer[])
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
      Serial.println("***ERROR: D2PIO_Init write failed");
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
        Serial.println("***ERROR: D2PIO_ReadBlocking timeout!");
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

  //D2PIO_Dump("D2PIO << ", buffer);
  return true;
}
//=============================================================================
// D2PIO_ReadMeasurement() Function
//=============================================================================
bool GDXLib::D2PIO_ReadMeasurement(byte buffer[], int timeout, float& measurement)
{
  #if defined DEBUG
   Serial.print("in D2PIO_ReadMeasurement ");
  #endif
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
    // Now that we have started received a measurement, we must wait for all of it to arrive.
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
    measurement = record;
    #if defined DEBUG
      Serial.print("***measurement in readMeasurement: ");
      Serial.println(measurement);
    #endif
  }
  else if (buffer[4] == NGI_BLOB_MEAS_BLOB_SUB_TYPE_WIDE_REAL32)
  {
    float record;
    memcpy(&record, &buffer[11], 4);
    measurement = record;
  }
  else if (buffer[4] == NGI_BLOB_MEAS_BLOB_SUB_TYPE_SINGLE_CHANNEL_INT32)
  {
    int32_t recordI32 = 0;
    memcpy(&recordI32, &buffer[8], 4);
    measurement = recordI32;
  }
  else if (buffer[4] == NGI_BLOB_MEAS_BLOB_SUB_TYPE_APERIODIC_INT32)
  {
    int32_t recordI32 = 0;
    memcpy(&recordI32, &buffer[8], 4);
    measurement = recordI32;
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
    Serial.print("***  Master FW version: ");
    Serial.print(pResponse->majorVersionMasterCPU);
    Serial.print("***.");
    Serial.print(pResponse->minorVersionMasterCPU);
    Serial.print("***.");
    Serial.println(pResponse->buildNumMasterCPU);
    Serial.print("***  Slave FW version: ");
    Serial.print(pResponse->majorVersionSlaveCPU);
    Serial.print("***.");
    Serial.print(pResponse->minorVersionSlaveCPU);
    Serial.print("***.");
    Serial.println(pResponse->buildNumSlaveCPU);
    Serial.print("*** in D2PIO_GetStatus Battery percent: ");
    Serial.println(pResponse->batteryLevelPercent);
    Serial.print("***  batteryLevelPercent: ");
    Serial.print("***  Charger state: ");
    Serial.println(pResponse->chargerState);
  #endif
  chargerStatus= (pResponse->chargerState);
  return true;
}
//=============================================================================
// D2PIO_GetDeviceInfo() Function
//=============================================================================
bool GDXLib::D2PIO_GetDeviceInfo()
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_GetDviceInfo() Function");
   #endif
   byte command[] = {
    0x58, 0x00, 0x00, 0x00, D2PIO_CMD_ID_GET_DEVICE_INFO
  };

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;

  D2PIOGetDeviceInfoCmdResponse* pResponse;
  pResponse = (D2PIOGetDeviceInfoCmdResponse*)&g_ReadBuffer[6];
  memcpy(&g_deviceInfo, pResponse, sizeof(D2PIOGetDeviceInfoCmdResponse));

  #if defined DEBUG
  Serial.println("***Device info:");
  Serial.print("***  Description: ");
  Serial.println(pResponse->DeviceDescription);
  Serial.println(pResponse->SerialNumber);
  Serial.print("***  Device name: ");
  Serial.println(pResponse->DeviceName);
  Serial.print("***  Mfg ID: ");
  Serial.println(pResponse->manufacturerId);
  Serial.print("***  Mfg Date: ");
  Serial.print(pResponse->ManufacturedMonth);
  Serial.print("***");
  Serial.print(pResponse->ManufacturedDay);
  Serial.print("***");
  Serial.println(pResponse->manufacturedYear);
  Serial.print("***  Master FW version: ");
  Serial.print(pResponse->majorVersionMasterCPU);
  Serial.print("***.");
  Serial.print(pResponse->minorVersionMasterCPU);
  Serial.print("***.");
  Serial.println(pResponse->buildNumMasterCPU);
  Serial.print("***  Slave FW version: ");
  Serial.print(pResponse->majorVersionSlaveCPU);
  Serial.print("***.");
  Serial.print(pResponse->minorVersionSlaveCPU);
  Serial.print("***.");
  Serial.println(pResponse->buildNumSlaveCPU);
  Serial.print("***  BLE address: ");
  Serial.print(pResponse->BLE_Address[5], HEX);
  Serial.print("***:");
  Serial.print(pResponse->BLE_Address[4], HEX);
  Serial.print("***:");
  Serial.print(pResponse->BLE_Address[3], HEX);
  Serial.print("***:");
  Serial.print(pResponse->BLE_Address[2], HEX);
  Serial.print("***:");
  Serial.print(pResponse->BLE_Address[1], HEX);
  Serial.print("***:");
  Serial.println(pResponse->BLE_Address[0], HEX);
  Serial.print("***  NVRAM size: ");
  Serial.println(pResponse->NVMemSize);
   #endif
  return true;
}
//=============================================================================
// D2PIO_GetChannelInfo() Function
//=============================================================================
bool GDXLib::D2PIO_GetChannelInfo(byte channelNumber)
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

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;

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
  return true;
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
      if (!D2PIO_GetChannelInfo(i)) return false;
    }
    testMask = testMask << 1;
   }
   return true;
}
//=============================================================================
// D2PIO_Autoset() Function
//=============================================================================
bool GDXLib::D2PIO_Autoset()
{
   #if defined DEBUG
      Serial.println ("***in D2PIO_Autoset() Function");
   #endif
  unsigned long availableMask = 0;
  unsigned long defaultMask = 0;
  unsigned long testMask = 1;
  byte i;
  boolean useDefaultChannel=0;//flag set on whether we want to use default channel
  if (g_channelNumber==255) useDefaultChannel=1;//channelNumber is set this way to show we want default channel
  if (!D2PIO_GetAvailableChannels(availableMask)) return false;
  if (!D2PIO_GetDefaultChannels(defaultMask)) return false;
  // Select the first channel number that is called out in the default mask
  for (i = 0; i < 32; i++)
  {
    if (testMask & defaultMask & availableMask)
    {
      if (useDefaultChannel) g_channelNumber = i;//use the default channel
      break;
    }
    testMask = testMask << 1;
  }
  if (i == 32) return false;

  #if defined DEBUG
    Serial.print(" ***g_channelNumber ");
    Serial.print(g_channelNumber);
  #endif
     if ((_deviceName[4] == 'S')&&
         (_deviceName[5] == 'N')&&
         (_deviceName[6] == 'D')){
             g_channelNumber=2;//set for A-weighted dB for Sound Sensor
         #if defined DEBUG
            Serial.print(" ***g_channelNumber ");
            Serial.println(g_channelNumber);
         #endif
         }   
  // Get the channel info
  if (!D2PIO_GetChannelInfo(g_channelNumber)) return false;
  
  // Set the sample rate according to the typical value for this sensor.
  // However we limit it to about 200ms for the sake of Arduino.
  // Not sure if this is actually slow enough!?
  g_samplePeriodInMilliseconds = g_channelInfo.typMeasurementPeriod / 1000;
  //WE MAY WANT TO REDUCE THE TIME OR GET RID OF LATER!!!
  if (g_samplePeriodInMilliseconds < 200) g_samplePeriodInMilliseconds = 200;

  #if defined DEBUG
    Serial.print("***Autoset channel number: ");
    Serial.println(g_channelNumber);
    Serial.print("***Autoset sample period (ms): ");
    Serial.println(g_samplePeriodInMilliseconds);
  #endif
  return true;
}
//=============================================================================
// D2PIO_StartMeasurements() Function
//=============================================================================
bool GDXLib::D2PIO_StartMeasurements(byte channelNumber)
{
   #if defined DEBUG
      Serial.println ("***@@@ in D2PIO_StartMeasurement() Function");
   #endif
   byte command[] = {
    0x58, 0x00, 0x00, 0x00, 0x18,
    0xFF,
    0x01,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

  // Convert the channel number to a bitmask and populate the payload
  unsigned long channelMask = 1;
  channelMask = channelMask << channelNumber;
  command[7]  = (channelMask >> 0)  & 0xFF;
  command[8]  = (channelMask >> 8)  & 0xFF;
  command[9]  = (channelMask >> 16) & 0xFF;
  command[10] = (channelMask >> 24) & 0xFF;

  // Populate the packet header bytes
  command[1] = sizeof(command);
  command[2] = g_rollingCounter--;
  command[3] = D2PIO_CalculateChecksum(command);

  if (!D2PIO_Write(command)) return false;
  if (!D2PIO_ReadBlocking(g_ReadBuffer, 5000)) return false;  
  return true;
}
//end of D2PIO functions

///=============================================================================
// GoDirectBLE_Error() Function// Just added 041920
//=============================================================================
void GDXLib::GoDirectBLE_Error()
{
  Serial.println("***Error, in the GoDirectBLE_Error Function");
  delay(1000);
  BLE.begin();
  #if defined DEBUG
    Serial.println("***BLE reset going on");
    // Cleanup any old connections //Kevin's reset
    Serial.print("***trying to reconnect...");
    Serial.print("g_autoConnect ");
    Serial.println(g_autoConnect);
  #endif
  delay(1000);

  if (BLE.connected())
    BLE.disconnect();
  delay(1000);
  if (g_autoConnect)
    BLE.scan(true);
  else
    BLE.scanForName(g_deviceName, true);
}
//=============================================================================
// open() Function
//=============================================================================
void GDXLib::open()  // This used to be labelled Begin
{
  #if defined DEBUG
    Serial.println("***in open(GDX....");
  #endif
  g_deviceName = NULL;
  g_channelNumber = 255;//this will cause the default channel to be used
  g_samplePeriodInMilliseconds = 0;
  g_autoConnect = true;//this will mean searching for any GDX device
  GoDirectBLE_Scan();

} //end open
//=============================================================================
// open(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds) Function
//=============================================================================
void GDXLib::open(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds)
{
  #if defined DEBUG
    Serial.println("***in open(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds)");
    Serial.print("deviceName ");
    Serial.println(deviceName);
  #endif
  g_deviceName = deviceName;
  g_channelNumber = channelNumber;
  g_samplePeriodInMilliseconds = samplePeriodInMilliseconds;
  if (deviceName[3]=='*') 
       g_autoConnect = true;//this allow for the search for any GDX device
  else g_autoConnect = false;// search for one specific GDX device
  //Serial.print("autoConnect = ");
  //Serial.println(g_autoConnect);
  #if defined DEBUG
    Serial.print("*** searching for "); 
    Serial.println(deviceName);
    Serial.print("***g_channelNumber specified ");
    Serial.println(g_channelNumber);
    Serial.print("***deviceName");
    Serial.println(deviceName);  
  #endif
  GoDirectBLE_Scan();
  } //end open


//=============================================================================
// GoDirectBLE_Scan() Function
//=============================================================================
  void GDXLib::GoDirectBLE_Scan()
  {
    Serial.println("Initializing ...");
    if (!BLE.begin()) {
      Serial.println("starting Bluetooth® Low Energy module failed");
      Serial.println("Disconnect USB cable, reconnect, and run the Upload again");
      while (1);
    }

    Serial.println("Begin BLE Scan for Go Direct sensor");
    //BLE.scanForName(Str5, true);
    //BLE.scanForName(Str5);
    //BLE.scanForName("GDX-HD 151000C1");
    BLE.scanForName(g_deviceName);
    Serial.print("scan for name: ");
    Serial.println(g_deviceName);
    Serial.println();
    delay(1000);

    int scanResult = D2PIO_SCAN_RESULT_NONE; //0

    Serial.println("Discovering ...");
    // loop until a peripheral is found
    while (true) {
      BLEDevice peripheral = BLE.available();
      if (peripheral) {     //escape while loop if found
        // discovered a peripheral
        Serial.println("Discovered a peripheral! Scan stopped");
        BLE.stopScan();
        g_peripheral = peripheral;
        break;
      }
    delay(500);
    Serial.println("No peripheral, Scan again");
    }

    scanResult == D2PIO_SCAN_RESULT_SUCCESS;
  
    Serial.print("Address: ");
    Serial.println(g_peripheral.address());
    Serial.print("Local Name: ");
    Serial.println(g_peripheral.localName());
    Serial.print("RSSI: ");
    Serial.println(g_peripheral.rssi());
    Serial.println();


    Serial.println("Connecting ...");
  
    if (g_peripheral.connect()) {
      Serial.println("Connected");
      Serial.println();
    } else {
      Serial.println("Failed to connect!");
      Serial.println("Disconnect USB cable, reconnect, and run the Upload again");
      while (1);
    }

    // discover peripheral attributes
    Serial.println("Discovering attributes ...");
    if (g_peripheral.discoverAttributes()) {
      Serial.println("Attributes discovered");
    } else {
      Serial.println("Attribute discovery failed!");
      Serial.println("Disconnect USB cable, reconnect, and run the Upload again");
      g_peripheral.disconnect();
      //BLE.end(); Jorge only suggested peripheral.disonnect()
      while (1);
    }

    // read and print device name of peripheral
    Serial.println();
    Serial.println("Connected Device Name...");
    Serial.println(g_peripheral.deviceName());
    Serial.println();



    delay(10);  // Kevin: seems okay without this delay//!!!
    if (!D2PIO_DiscoverService(g_peripheral)) //Kevin's Discover 
      GoDirectBLE_Error();
    if (!D2PIO_Init())
      GoDirectBLE_Error();

    // Wait for connection interval to finish negotiating
    delay(1000);
  
    #if defined DEBUG
      Serial.println("");
      Serial.print("*** g_deviceName");
      Serial.println(g_deviceName);
    #endif
  
    if (!D2PIO_GetStatus())
      GoDirectBLE_Error();
    if (!D2PIO_GetDeviceInfo()) //Kevin's Setup
      GoDirectBLE_Error();
    if (!D2PIO_GetChannelInfoAll())
      GoDirectBLE_Error();
    
    if (!D2PIO_Autoset())//select default channel
        GoDirectBLE_Error();

    if (!D2PIO_GetChannelInfo(g_channelNumber))
          GoDirectBLE_Error();

    if (!D2PIO_SetMeasurementPeriod(g_samplePeriodInMilliseconds))
      GoDirectBLE_Error();
    //below is the AutoID code, which really just reports:

    //below is the former AutoID code, which really sets values
    _RSSI=GoDirectBLE_GetScanRSSI(); 
    _batteryPercent=GoDirectBLE_GetBatteryStatus();
    _chargeState   =GoDirectBLE_GetChargeStatus();
    _samplePeriodInMilliseconds =GoDirectBLE_GetSamplePeriod();
    _channelNumber =GoDirectBLE_GetChannelNumber();
    sprintf(_channelName,"%s",GoDirectBLE_GetChannelName());
    sprintf(_deviceName,"%s",GoDirectBLE_GetDeviceName());
    sprintf(_orderCode,"%s",GoDirectBLE_GetOrderCode());
    sprintf(_serialNumber,"%s",GoDirectBLE_GetSerialNumber());
    sprintf(_channelUnits,"%s",GoDirectBLE_GetChannelUnits());

    #if defined DEBUG
    Serial.println("***HERE is all the info");
    Serial.print("*** _RSSI"); 
    Serial.println(_RSSI);
    Serial.print("*** _batteryPercent");
    Serial.println(_batteryPercent);
    Serial.print("***_chargeState");
    Serial.println(_chargeState);
    Serial.print("***_channelUnits");
    Serial.println(_channelUnits);
    Serial.print("***_channelName");
    Serial.println(_channelName);
    Serial.print("***_deviceName");
    Serial.println(_deviceName);
    #endif

    }  //end of while





//=============================================================================
// GoDirectBLE_Scan() Function
//=============================================================================
  /*void GDXLib::GoDirectBLE_Scan()
  {
  BLE.begin();
  #if defined DEBUG
    Serial.print("***BLE reset");
  #endif
  // Cleanup any old connections //Kevin's reset
  Serial.print("is BLE Connected? ");
  if (BLE.connected())
    {
    BLE.disconnect();
    Serial.println("yes");
    }
  else
    Serial.println("no");
  //start scanning:
  if (g_autoConnect)
    {
    BLE.scan(true);
    Serial.println("regular scan for any device");
    }
  else
    {
    BLE.scanForName(g_deviceName, true);
    Serial.print("scan for name: ");
    Serial.println(g_deviceName);
    }

  int scanResult = D2PIO_SCAN_RESULT_NONE; //0
  //Kevin's Idle through Flush

  while (scanResult != D2PIO_SCAN_RESULT_SUCCESS) //3
  {
    scanResult = D2PIO_Scan(g_autoConnect, GDX_BLE_AUTO_CONNECT_RSSI_THRESHOLD);
    Serial.print("scanResult =  ");
    Serial.println(scanResult);
    if (scanResult == D2PIO_SCAN_RESULT_SUCCESS) //3
     {  
      #if defined DEBUG
        Serial.println("***D2PIO_SCAN_RESULT_SUCCESS");
     #endif
      break;
      }//end of if
    if (scanResult == D2PIO_SCAN_RESULT_WEAK) //1
      Serial.println(".");
      //Serial.println("***D2PIO_SCAN_RESULT_WEAK");
    if (scanResult == D2PIO_SCAN_RESULT_FLUSH) 
      Serial.println("-");
      //Serial.println("D2PIO_SCAN_RESULT_FLUSH");
  }//end while
  
  // Stop scanning
  BLE.stopScan();
  #if defined DEBUG
     Serial.print("***Connecting...");
  #endif
  if (!g_peripheral.connect())
  {
    Serial.println("FAILED!");
    GoDirectBLE_Error();
  }//end if
  else
  {
   #if defined DEBUG
      Serial.println("SUCCESS");
   #endif
   delay(10);  // Kevin: seems okay without this delay//!!!
  if (!D2PIO_DiscoverService(g_peripheral)) //Kevin's Discover 
    GoDirectBLE_Error();
  if (!D2PIO_Init())
    GoDirectBLE_Error();

  // Wait for connection interval to finish negotiating
  delay(1000);
  
  #if defined DEBUG
    Serial.println("");
    Serial.print("*** g_deviceName");
    Serial.println(g_deviceName);
  #endif
   
  if (!D2PIO_GetStatus())
    GoDirectBLE_Error();
  if (!D2PIO_GetDeviceInfo()) //Kevin's Setup
    GoDirectBLE_Error();
  if (!D2PIO_GetChannelInfoAll())
    GoDirectBLE_Error();
    
   if (!D2PIO_Autoset())//select default channel
      GoDirectBLE_Error();
 
  if (!D2PIO_GetChannelInfo(g_channelNumber))
        GoDirectBLE_Error();

  if (!D2PIO_SetMeasurementPeriod(g_samplePeriodInMilliseconds))
    GoDirectBLE_Error();
  //below is the AutoID code, which really just reports:

  //below is the former AutoID code, which really sets values
  _RSSI=GoDirectBLE_GetScanRSSI(); 
  _batteryPercent=GoDirectBLE_GetBatteryStatus();
  _chargeState   =GoDirectBLE_GetChargeStatus();
  _samplePeriodInMilliseconds =GoDirectBLE_GetSamplePeriod();
  _channelNumber =GoDirectBLE_GetChannelNumber();
  sprintf(_channelName,"%s",GoDirectBLE_GetChannelName());
  sprintf(_deviceName,"%s",GoDirectBLE_GetDeviceName());
  sprintf(_orderCode,"%s",GoDirectBLE_GetOrderCode());
  sprintf(_serialNumber,"%s",GoDirectBLE_GetSerialNumber());
  sprintf(_channelUnits,"%s",GoDirectBLE_GetChannelUnits());

    #if defined DEBUG
    Serial.println("***HERE is all the info");
    Serial.print("*** _RSSI"); 
    Serial.println(_RSSI);
    Serial.print("*** _batteryPercent");
    Serial.println(_batteryPercent);
    Serial.print("***_chargeState");
    Serial.println(_chargeState);
    Serial.print("***_channelUnits");
    Serial.println(_channelUnits);
    Serial.print("***_channelName");
    Serial.println(_channelName);
    Serial.print("***_deviceName");
    Serial.println(_deviceName);
 #endif

  }//end of while

}//end of Scan  }
*/

 //=============================================================================
// start() Function
//=============================================================================!@
   void GDXLib::start() {
    #if defined DEBUG
    Serial.print("**$ calling _StartMeasurements, g_channel: ");
    Serial.println(g_channelNumber); 
  #endif
  if (!D2PIO_StartMeasurements(g_channelNumber))
    GoDirectBLE_Error();
   }

 //=============================================================================
// readSensor() Function
//=============================================================================!@
float GDXLib::readSensor() 
{
  #if defined DEBUG
        Serial.print("**in readSensor,  samplePeriodInMilliseconds) ");
        Serial.println(g_samplePeriodInMilliseconds);      
  #endif
  if (!BLE.connected())
     GoDirectBLE_Error();
  if(!D2PIO_ReadMeasurement(g_ReadBuffer, 5000, g_measurement)){
      #if defined DEBUG
        delay (5);//is there any reason for this? !!!
        //Serial.print("#"); 
      #endif
      }
  #if defined DEBUG
     Serial.print("*** g_measurement back in readSensor: ");
     Serial.println(g_measurement);
  #endif
  return g_measurement;
  }

//=============================================================================
// GoDirectBLE_GetStatus() Function//not used!!!
//=============================================================================
void GDXLib::GoDirectBLE_GetStatus(char* strFirmwareVersion1, char* strFirmwareVersion2, byte& batteryPercent)
{
  sprintf(strFirmwareVersion1, "%d.%d", g_status.majorVersionMasterCPU, g_status.minorVersionMasterCPU);
  sprintf(strFirmwareVersion2, "%d.%d", g_status.majorVersionSlaveCPU,  g_status.minorVersionSlaveCPU);

  batteryPercent = g_status.batteryLevelPercent;
  Serial.print("*** batteryPercent: ");
  Serial.println(batteryPercent);//this is correct here in the library code
}
//=============================================================================
// GoDirectBLE_GetDeviceName() Function
//=============================================================================
const char* GDXLib::GoDirectBLE_GetDeviceName()
{
return g_peripheral.localName().c_str();
//return g_peripheral.localName().c_str();
}

// GoDirectBLE_GetDeviceName() Function
//=============================================================================
//const char* GoDirectBLE_GetDeviceName()
//{
//  return g_peripheral.localName().c_str();
//}

//=============================================================================
// GoDirectBLE_GetChannelName() Function
//=============================================================================
const char* GDXLib::GoDirectBLE_GetChannelName()
{
  return g_channelInfo.sensorDescription;
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
// GoDirectBLE_GetChannelUnits() Function
//=============================================================================
const char* GDXLib::GoDirectBLE_GetChannelUnits()
{
  return g_channelInfo.sensorUnit;
}
//=============================================================================
// GoDirectBLE_GetSamplePeriod() Function
//=============================================================================
unsigned long GDXLib::GoDirectBLE_GetSamplePeriod()
{
  return g_samplePeriodInMilliseconds;
}
//=============================================================================
// GoDirectBLE_GetChannelNumber() Function
//=============================================================================
int GDXLib::GoDirectBLE_GetChannelNumber()
{
  return g_channelNumber;
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
// GoDirectBLE_GetMeasurement() Function NOT USED, BUT SHOULD IT BE?
//=============================================================================
float GDXLib::GoDirectBLE_GetMeasurement()
{
  return g_measurement;
}
//=============================================================================
// GoDirectBLE_End() Function 
//=============================================================================
void GDXLib::stop()
{
  //BLE.end(); Jorge only suggested peripheral.disconnect()
  // Also, this shold be used to stop data collection
  g_peripheral.disconnect();
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
