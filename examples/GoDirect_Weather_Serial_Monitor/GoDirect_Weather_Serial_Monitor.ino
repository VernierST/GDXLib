/*
In the example, 4 channels of the Go Direct Weather System are
configured (Wind Speed, Temperature, Relative Humidity, and Barometric
Pressure). The measurements are printed on the Serial Monitor.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow: 

  1. There is 1 important variable to configure.
    a. Set the 'myDevice' variable with the order code and serial 
      number of your device
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


// ****** IMPORTANT! INPUT YOUR DEVICE NAME ******
char* myDevice = "GDX-WTHR 13400106"; // your Go Direct name and serial number. For example, myDevice = "GDX-WTHR 13100004"

// variables to store the active channel numbers
byte sensor1 = 1; // Wind Speed is on sensor channel 1
byte sensor2 = 4; // Temperature is on sensor channel 4
byte sensor3 = 7; // Relative Humidity is on sensor channel  7
byte sensor4 = 10; // Barometric Pressure is on sensor channel 10

void setup(){
  Serial.begin(9600);
  delay(6000);
  Serial.print("Begin Search for WTHR");
  Serial.println();
  
  if (!GDX.open(myDevice)) {
    Serial.println("GDX.open() failed. Disconnect/Reconnect USB"); 
    while (true); //if open() fails, put the Arduino into a do-nothing loop
  }

  Serial.print("Successfully found: ");
  Serial.println(GDX.getDeviceName());
  Serial.println();
  
  GDX.enableSensor(sensor1); 
  GDX.enableSensor(sensor2); 
  GDX.enableSensor(sensor3); 
  GDX.enableSensor(sensor4);

  //print headers using the sensor's name
  Serial.print(GDX.getSensorName(sensor1));
  Serial.print(", ");
  Serial.print(GDX.getSensorName(sensor2));
  Serial.print(", ");
  Serial.print(GDX.getSensorName(sensor3));
  Serial.print(", ");
  Serial.println(GDX.getSensorName(sensor4));

  // start sampling with a period of 2000 milliseconds (2 seconds)
  GDX.start(2000);  
}

void loop(){  

  for (int i = 0; i < 10; i++) {
    GDX.read();
    Serial.print(GDX.getMeasurement(sensor1));
    Serial.print(", ");
    Serial.print(GDX.getMeasurement(sensor2));
    Serial.print(", ");
    Serial.print(GDX.getMeasurement(sensor3));
    Serial.print(", ");
    Serial.println(GDX.getMeasurement(sensor4));
  }

  Serial.println();
  Serial.println("data collection complete"); 
  Serial.println(); 
  GDX.stop();
  GDX.close();
  while(true); // collection done. Put Arduino into a do-nothing loop
}
