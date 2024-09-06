/*
In the example, sensor 1 is enabled and configured to collect 5 
samples at a period of 2000ms. Data collection is restarted after 
a pause of 5 seconds, with a sample period of 1000ms.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow: 

  1. In 'GDX.open()', put the name and serial number of your device
  2. In 'GDX.enableSensor()' put the sensor number that you wish to enable
  3. In 'GDX.getMeasurement()' put the sensor number of the enabled sensor
  4. Open the Serial Monitor after the Upload is done
  5. Turn on the Go Direct device

*/

#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

void setup(){
  Serial.begin(9600);
  delay(6000);

  if (!GDX.open("GDX-HD 151000C1")) {
    Serial.println("GDX.open() failed. Disconnect/Reconnect USB");
    while (true);  //if open() fails, put the Arduino into a do-nothing loop
  }

  GDX.enableSensor(1); 

  GDX.start(2000);  // sample period in milliseconds
  for (int i = 0; i < 5; i++) {
    GDX.read();
    Serial.println(GDX.getMeasurement(1));
   }
   GDX.stop();  //stop data collection

  Serial.println();
  Serial.println("Data collection stopped. Restart in 5 seconds");
  Serial.println();
  delay(5000);

  GDX.start(1000);  // restart data collection
  for (int i = 0; i < 5; i++) {
    GDX.read();
    Serial.println(GDX.getMeasurement(1));
   }
   GDX.stop();  //stop data collection
   GDX.close();
}

void loop(){
}