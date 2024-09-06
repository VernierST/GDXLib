/*
In the example, after the Go Direct device has been opened the
device name is printed to the Serial Monitor. This provides nice
feedback that the Arduino has found, and connected to, the Go 
Direct device via Bluetooth.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow: 

  1. In 'GDX.open()', put the name and serial number of your device
  2. Open the Serial Monitor after the Upload is done
  3. Turn on the Go Direct device

*/

#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

void setup(){
  Serial.begin(9600);
  delay(6000);

  if (!GDX.open("GDX-HD 151000C1")) {
      Serial.println("GDX.open() failed. Disconnect/Reconnect USB"); 
      while (true); //if open() fails, put the Arduino into a do-nothing loop
  }

  // If the device was opened properly, print the found device's name
  Serial.print("Found: ");
  Serial.println(GDX.getDeviceName());
  GDX.close();
}

void loop(){
}
