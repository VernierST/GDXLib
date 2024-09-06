/*
In the example, the sensor's name and units are used to create
a header for the data. The header and sensor data are printed to the 
Serial Monitor.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow:

  1. Set the 'sensor' variable with the sensor number to enable
  2. Set the 'device' variable with the name and serial number of your device
  3. Open the Serial Monitor after the Upload is done
  4. Turn on the Go Direct device

*/

#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

byte sensor = 1; // select the device's sensor to read. In most Go Direct devices, sensor 1 is the default
char* device = "GDX-HD 151000C1"; // put your Go Direct name and serial number here, between the quotes

void setup(){
  Serial.begin(9600);
  delay(6000);

  if (!GDX.open(device)) {
     Serial.println("GDX.open() failed. Disconnect/Reconnect USB");
    while (true);  //if open() fails, put the Arduino into a do-nothing loop
  }

  GDX.enableSensor(sensor); 

  //print headers using the enabled sensor's name and sensor units
  Serial.print(GDX.getSensorName(sensor));
  Serial.print(" ");
  Serial.println(GDX.getUnits(sensor));

  GDX.start(1000);  // sample period in milliseconds
  for (int i = 0; i < 10; i++) {
    GDX.read();
    Serial.println(GDX.getMeasurement(sensor));
  }
  GDX.stop();
  GDX.close();
}

void loop(){
}
