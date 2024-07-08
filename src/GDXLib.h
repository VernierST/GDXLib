  #ifndef GDXLib_h  
  #define GDXLib_h
  //define GDXLIB_LIB_VERSION "0.90" // displayed automatically
  // This is a library to make using GDX sensors easy
  #include "ArduinoBLE.h"

class GDXLib 
{
 public:
    GDXLib();//definition of the GDXLib class
    
    void autoID();//this is the function for the autoID code
    // it returns calibration information
    char* channelName()        { return _channelName ;};
    const char* deviceName()   { return _deviceName ;};
    const char* orderCode()    { return _orderCode ;};
    const char* serialNumber() { return _serialNumber ;};
    char* channelUnits()       { return _channelUnits ;};
    uint8_t batteryPercent()   { return _batteryPercent ;};
    uint8_t chargeState()      { return _chargeState ;};
    int RSSI()                 { return _RSSI ;};
    int channelNumber()        { return _channelNumber ;};
    unsigned long samplePeriodInMilliseconds() { return _samplePeriodInMilliseconds;};
    //void open();
    //bool open();
    //void open(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds);
    //bool open(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds);
    bool open(char* deviceName="proximity"); //default arguments added in .h file, not .cpp file
    void stop();
    void start();
    void close();
    float readSensor();//a public method
    bool D2PIO_ReadMeasurement(byte buffer[], int timeout, float& measurement);
    float GoDirectBLE_GetMeasurement();
    
 private:// 
    char _channelName[32]="channelName";
    char _deviceName[32]="deviceName";
    char _orderCode[16]="orderCode";
    char _serialNumber[16]="serialNumber";
    char _channelUnits[32]="channelUnits";
    uint8_t _batteryPercent=0;
    uint8_t _chargeState=0;
    int _RSSI=0;
    int _channelNumber=0;
    unsigned long _samplePeriodInMilliseconds;
    int  _channel;
    char _strBuffer[32]; //used in Read Sensor
    
    bool DumpGatttService(BLEDevice peripheral, char* uuid);
    bool D2PIO_DiscoverService(BLEDevice peripheral);
    byte D2PIO_CalculateChecksum(const byte buffer[]);
    void D2PIO_Dump(const char* strPrefix, const byte buffer[]);
    bool D2PIO_Write(const byte buffer[]);
    bool D2PIO_ReadBlocking(byte buffer[], int timeout);

    bool D2PIO_Init();
    bool D2PIO_SetMeasurementPeriod(unsigned long measurementPeriodInMilliseconds);
    bool D2PIO_GetAvailableChannels(unsigned long& availableMask);
    bool D2PIO_GetDefaultChannels(unsigned long& defaultMask);
    bool D2PIO_GetStatus();
    bool D2PIO_GetDeviceInfo();
    bool D2PIO_GetChannelInfo(byte channelNumber);
    bool D2PIO_GetChannelInfoAll();
    bool D2PIO_Autoset();
    bool D2PIO_StartMeasurements(byte channelNumber);
    
    void GoDirectBLE_Measure();
    void GoDirectBLE_Error();
    void GoDirectBLE_Start();
    bool GoDirectBLE_Scan_For_Name();
    bool GoDirectBLE_Scan_Proximity();
    bool GoDirectBLE_Connect();
    bool GoDirectBLE_Discover_Attributes();
    void GoDirectBLE_Reset();
    void GoDirectBLE_Read();
    void GoDirectBLE_GetStatus(char* strFirmwareVersion1, char* strFirmwareVersion2, byte& batteryPercent);
    int GoDirectBLE_GetScanRSSI();
    const char* GoDirectBLE_GetDeviceName();
    const  char* GoDirectBLE_GetChannelName();
    const char* GoDirectBLE_GetSerialNumber();
    const char* GoDirectBLE_GetOrderCode();
    const char* GoDirectBLE_GetChannelUnits();
    uint8_t GoDirectBLE_GetBatteryStatus();
    uint8_t GoDirectBLE_GetChargeStatus();
    int    GoDirectBLE_GetRSSI();
    unsigned long GoDirectBLE_GetSamplePeriod();
    bool GoDirectBLE_DisplayChannelAsInteger();
    int GoDirectBLE_GetChannelNumber();
};
#endif
