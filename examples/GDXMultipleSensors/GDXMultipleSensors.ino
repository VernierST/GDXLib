/*
In the example, sensor 1, 2, and 3 are enabled and configured 
to collect 10 samples at a period of 1000ms. The data are printed 
to the Serial Monitor.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow: 

  1. In 'GDX.open()', put the name and serial number of your device
  2. In 'GDX.enableSensor()' put the sensor numbers that you wish to enable
  3. In 'GDX.getSensorName()' put the sensor numbers of the enabled sensors
  4. In 'GDX.getMeasurement()' put the sensor numbers of the enabled sensors
  5. Open the Serial Monitor after the Upload is done
  6. Turn on the Go Direct device

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

  GDX.enableSensor(1);
  GDX.enableSensor(2);
  GDX.enableSensor(3);

  //print headers using the sensor's name
  Serial.print(GDX.getSensorName(1));
  Serial.print(", ");
  Serial.print(GDX.getSensorName(2));
  Serial.print(", ");
  Serial.println(GDX.getSensorName(3));

  GDX.start(1000);  // sample period in milliseconds
  for (int i = 0; i < 10; i++) {
    GDX.read();
    Serial.print(GDX.getMeasurement(1));
    Serial.print(", ");
    Serial.print(GDX.getMeasurement(2));
    Serial.print(", ");
    Serial.println(GDX.getMeasurement(3));
  }
  GDX.stop();
  GDX.close();
}

void loop(){    
}
