/*
In the example, the specified sensor of the Go Direct device controls
the Arduino's built-in LED. The program starts with the built-in LED off. 
The built-in LED stays off until the Go Direct sensor begins data collection. 
Once data collection begins, the built-in LED will either blink rapidly
to indicate that the sensor reading is above the threshold, or stay on
without blinking if the reading is below the threshold.

If the LED never turns on, that indicates that the program did not 
properly connect with the Go Direct device and start data collection.
Make sure it is powered on. Disconnect and reconnect the USB cable and try again.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow:

  1. Set the 'sensor' variable with the sensor number to enable
  2. Set the 'device' variable with the name and serial number of your device
  3. Set the 'threshold' variable to a value that allows you to have sensor 
     readings that can go above and below it, in order to change the state of the LED
  4. Upload the program
  5. Turn on the Go Direct device

Of special note:

  Data collection is coded to loop indefinitely. There is no stop() or close()
  functions called. This means that when the Arduino program is terminated, the
  Bluetooth module will not have been shut down gracefully. This can result
  in the next run or the next upload to fail at GDX.open(). This is usually
  resolved by disonnecting and then reconnecting the USB cable from the Arduino.

*/

#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

byte sensor = 1; // select the device's sensor to read
char* device = "GDX-HD 151000C1"; // put your Go Direct name and serial number here, between the quotes
float threshold = 20; // modify this value as needed for your sensor

void setup(){
  Serial.begin(9600);
  delay(4000);
  
  pinMode(LED_BUILTIN, OUTPUT);

  if (!GDX.open(device)) {
    Serial.println("GDX.open() failed. Disconnect/Reconnect USB");
    while (true);  //if open() fails, put the Arduino into a do-nothing loop                     
    }

  GDX.enableSensor(sensor); 
  GDX.start(200);  // sample period in milliseconds
}

void loop(){
  GDX.read();
  float sensorValue = GDX.getMeasurement(sensor);
  if (sensorValue > threshold) {
    digitalWrite(LED_BUILTIN, HIGH);  
    delay(100);                      
    digitalWrite(LED_BUILTIN, LOW);   
    delay(100);
  }
  else {
    digitalWrite(LED_BUILTIN, HIGH); 
  }
}
