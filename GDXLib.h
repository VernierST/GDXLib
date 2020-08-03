  #ifndef GDXLib_h  
  #define GDXLib_h
  //define GDXLIB_LIB_VERSION "0.82" // displayed automatically
  // This is a library to make using GDX sensors easy
  #include "ArduinoBLE.h"

class GDXLib 
{
 public:
    GDXLib();//definition of the GDXLib class
    
    void autoID();//this is the function for the autoID code
    // it returns calibration information
    char* channelName()     { return _channelName ;};
    char* deviceName()       { return _deviceName ;};
    char* channelUnits()     { return _channelUnits ;};
    uint8_t batteryPercent() { return _batteryPercent ;};
    uint8_t chargeState()    { return _chargeState ;};
    int RSSI()               { return _RSSI ;};
    
    void Begin();
    void Begin(char* deviceName, byte channelNumber, unsigned long samplePeriodInMilliseconds);
    void Stop();
    float readSensor();//a public method
    bool D2PIO_ReadMeasurement(byte buffer[], int timeout, float& measurement);
    float GoDirectBLE_GetMeasurement();

    
 private:// also known as local  

    char _channelName[32];
    char _deviceName[32];
    char _channelUnits[32];// 32 bytes!!!
    uint8_t _batteryPercent;
    uint8_t _chargeState;
    int _RSSI;
    
    unsigned long _samplePeriodInMilliseconds;// used in begin
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
    bool D2PIO_GetChannelInfo(byte channelNumber, bool verbose);
    bool D2PIO_GetChannelInfoAll();
    bool D2PIO_Autoset();
    bool D2PIO_StartMeasurements(byte channelNumber);
    
    void GoDirectBLE_Measure();  //
    void GoDirectBLE_Error();
    void GoDirectBLE_Start();
    void GoDirectBLE_Scan();
    void GoDirectBLE_Reset();  //
    void GoDirectBLE_Read();
    void GoDirectBLE_GetStatus(char* strFirmwareVersion1, char* strFirmwareVersion2, byte& batteryPercent);
    int GoDirectBLE_GetScanRSSI();
    const char* GoDirectBLE_GetDeviceName();
    const char* GoDirectBLE_GetSerialNumber();
    const char* GoDirectBLE_GetOrderCode();
    const char* GoDirectBLE_GetChannelUnits();
    uint8_t GoDirectBLE_GetBatteryStatus();
    uint8_t GoDirectBLE_GetChargeStatus();
    int    GoDirectBLE_GetRSSI();
    bool GoDirectBLE_DisplayChannelAsInteger();
    char* GoDirectBLE_GetChannelName();
};
#endif
