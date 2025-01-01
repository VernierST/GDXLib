/*
In the example, 4 channels of the Go Direct Weather System are
configured (Wind Speed, Temperature, Relative Humidity, and Barometric
Pressure). The data are printed to the Serial Monitor.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow: 

  1. In 'GDX.open()', put the name and serial number of your device
  2. Make sure your Arduino is connected via USB and detected and 
  selected in the IDE
  3. Upload the program
  4. When the upload is done, click the Serial Monitor button to open 
  the Serial Monitor
  5. Turn on your Go Direct WTHR
  6. When done, close the Serial Monitor

Note: The Go Direct Weather has the following sensor channels

sensor number = 1: Wind Speed m/s
sensor number = 2: Wind Direction °
sensor number = 3: Wind Chill °C
sensor number = 4: Temperature °C
sensor number = 5: Heat Index °C
sensor number = 6: Dew Point °C
sensor number = 7: Relative Humidity %
sensor number = 8: Absolute Humidity g/m³
sensor number = 9: Station Pressure mbar
sensor number = 10: Barometric Pressure mbar
sensor number = 11: Altitude m

*/

#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

void setup(){
  Serial.begin(9600);
  delay(6000);
  
  // **IMPORTANT! CONFIGURE YOUR DEVICE (such as, GDX.open("GDX-WTHR 13400106"))**
  if (!GDX.open("GDX-WTHR 13400106")) {
      Serial.println("GDX.open() failed. Disconnect/Reconnect USB"); 
      while (true); //if open() fails, put the Arduino into a do-nothing loop
  }

  GDX.enableSensor(1);
  GDX.enableSensor(4);
  GDX.enableSensor(7);
  GDX.enableSensor(10);

  //print headers using the sensor's name
  Serial.print(GDX.getSensorName(1));
  Serial.print(", ");
  Serial.print(GDX.getSensorName(4));
  Serial.print(", ");
  Serial.print(GDX.getSensorName(7));
  Serial.print(", ");
  Serial.println(GDX.getSensorName(10));

  GDX.start(1000);  // sample period in milliseconds
  for (int i = 0; i < 10; i++) {
    GDX.read();
    Serial.print(GDX.getMeasurement(1));
    Serial.print(", ");
    Serial.print(GDX.getMeasurement(4));
    Serial.print(", ");
    Serial.print(GDX.getMeasurement(7));
    Serial.print(", ");
    Serial.println(GDX.getMeasurement(10));
  }
  GDX.stop();
  GDX.close();
}

void loop(){    
}
