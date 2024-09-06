  #ifndef GDXLib_h  
  #define GDXLib_h
  //define GDXLIB_LIB_VERSION "2.0.0" // displayed automatically
  // This is a library to make using GDX sensors easy
  #include "ArduinoBLE.h"

class GDXLib 
{
 public:
    GDXLib();//definition of the GDXLib class
    
    //void autoID();//this is the function for the autoID code
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
    //unsigned long samplePeriodInMilliseconds() { return _samplePeriodInMilliseconds;};
    //void open();
    //bool open();
    //void open(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds);
    //bool open(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds);
    bool open(char* deviceName); //="proximity" if they want to do proximity pairing
    void enableSensor(byte selectedSensor); //=255, if the user wants to use the default sensor
    void start(unsigned long period);
    void stop();
    void close();
    //float readSensor();//a public method
    void read();
    bool GDX_ReadMeasurement(byte buffer[], int timeout);
    byte GDX_getDefaultSensor();
    float getMeasurement(byte selectedSensor);
    //char getUnits(byte selectedSensor);
    const char* getUnits(byte selectedSensor);
    const char* getSensorName(byte selectedSensor);
    //const char* getDeviceName();
    String getDeviceName();
    
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
    //unsigned long _samplePeriodInMilliseconds;
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
    void D2PIO_GetChannelInfo(byte channelNumber);
    bool D2PIO_GetChannelInfoAll();
    bool GDX_StartMeasurements(unsigned long sensorMask);
    bool GDX_StopMeasurements();
   
    bool GoDirectBLE_Scan_For_Name();
    bool GoDirectBLE_Scan_Proximity();
    bool GoDirectBLE_Connect();
    bool GoDirectBLE_Discover_Attributes();
    int GoDirectBLE_GetScanRSSI();
    const char* GoDirectBLE_GetSerialNumber();
    const char* GoDirectBLE_GetOrderCode();
    uint8_t GoDirectBLE_GetBatteryStatus();
    uint8_t GoDirectBLE_GetChargeStatus();
    bool GoDirectBLE_DisplayChannelAsInteger();
};
#endif
