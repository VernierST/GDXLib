/*
In the example, a Go Direct device is configured to collect data from
a specified sensor, at a specified rate and duration. The Serial
Monitor is used for feedback, and to display the sensor measurement.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow:

  1. There are 4 important variables to configure. Locate these in the code
  below and set them appropriately for your device and experimnent.
    a. Set the 'myDevice' variable with the order code and serial number of your device
    b. Set the 'sensor' variable with the sensor number to enable
    c. Set the 'sampleRate' for the experiment (samples/second)
    c. Set the 'duration' of the experiment (seconds)
  2. Make sure your Arduino is connected via USB and detected and selected in the IDE
  3. Upload the program
  4. When the upload is done, click the Serial Monitor button to open the Serial Monitor
  5. Turn on your Go Direct device
  6. When done, close the Serial Monitor

*/

#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;


// ****** IMPORTANT! INPUT YOUR DEVICE NAME ******
char* myDevice = "GDX-HD 151000C1"; // your Go Direct name and serial number. For example, myDevice = "GDX-HD 151000C1"
byte sensor = 1; // select the device's sensor to read. In most devices, the default sensor is 1

// ****** Data Collection Parameters ******
int sampleRate = 1; // set the data collection sampling rate (samples/second)
int duration = 10; // set the data collection duration (seconds)

// Code to convert the sampleRate to sample period (in seconds and ms) and calculate total numbef of data points
float periodSec = 1.0/sampleRate; // convert the sample rate (samples/sec) to sample period (seconds)
unsigned long periodMs = periodSec * 1000; // convert sample period in seconds to sample period in milliseconds
int numPointsToCollect = sampleRate * duration + 1; // add 1 because the first point is collected at time = 0

void setup(){
  Serial.begin(9600);
  delay(4000);
  Serial.print("Search for: ");
  Serial.println(myDevice);

  if (!GDX.open(myDevice)) {
    Serial.println("GDX.open() failed. Disconnect/Reconnect USB");
    while (true);  //if open() fails, put the Arduino into a do-nothing loop                     
    }

  Serial.print("Successfully found: ");
  Serial.println(GDX.getDeviceName());
  Serial.println();

  GDX.enableSensor(sensor); 

  //print headers "Time(sec), SensorName(Units)"
  Serial.print("Time(sec)");
  Serial.print(" , ");
  Serial.print(GDX.getSensorName(sensor));
  Serial.print(" ");
  Serial.println(GDX.getUnits(sensor));

  //start sampling at the specified sample period. Note this is in milliseconds
  GDX.start(periodMs);  
}

void loop(){
  int points = 1;  //variable to keep track of how many pts have been collected
  float time = 0;  //variable to keep track of how much time (in seconds) has elapsed

  while (points <= numPointsToCollect){
    GDX.read();
    Serial.print(time);
    Serial.print(" , ");
    float sensorValue = GDX.getMeasurement(sensor);
    Serial.println(sensorValue);
    time = time + periodSec;
    points ++;
  }
  
  Serial.println();
  Serial.println("data collection complete"); 
  Serial.println(); 
  GDX.stop(); 
  GDX.close();
  while(true); // collection done. Put Arduino into a do-nothing loop
}
