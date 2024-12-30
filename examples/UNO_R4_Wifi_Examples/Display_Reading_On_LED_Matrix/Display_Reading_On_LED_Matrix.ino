/*
In the example, a Go Direct device is configured to collect data from
a specified sensor, at a specified rate and duration. The LED Matrix
on the UNO R4 Wifi is used for feedback, and to report the data. 

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
  4. When the upload is done, look for the Bluetooth icon on the LED Matrix, this 
  means it is searching for the Go Direct device.
  5. Turn on your Go Direct device
  6. The sensor measurments will be displayed on the LED Matrix

 */


#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

// To use ArduinoGraphics APIs, the #include must come BEFORE ArduinoLEDMatrix
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;

 
// ****** IMPORTANT! CONFIGURE YOUR DEVICE ******
char* myDevice = "GDX-HD 151000C1"; // put the Go Direct name and serial number here. For example, myDevice = "GDX-HD 151000C1"
byte sensor = 1; // select the device's sensor to read. In most devices, the default sensor is 1

// ****** IMPORTANT! CONFIGURE DATA COLLECTION ******
int sampleRate = 1; // set the data collection sampling rate (samples/second)
int duration = 10; // set the data collection duration (seconds)


float periodSec = 1.0/sampleRate; // convert the sample rate (samples/sec) to sample period (seconds)
unsigned long periodMs = periodSec * 1000; // convert sample period (seconds) to sample period (milliseconds)
int numPointsToCollect = sampleRate * duration + 1; // add 1 because the first point is collected at time = 0
String msg; // variable to store the measurement
String nameFound;

void setup(){

  // show the bluetooth icon on the LED Matrix as it scans for the Go Direct device
  matrix.begin();
  matrix.loadFrame(LEDMATRIX_BLUETOOTH);
  delay(1000);

  if (!GDX.open(myDevice)) {
    Serial.println("GDX.open() failed. Disconnect/Reconnect USB");
    while (true);  //if open() fails, put the Arduino into a do-nothing loop 
  }

  // after GDX.open() succeeds, scroll the device name on the LED Matrix
  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(50);
  nameFound = GDX.getDeviceName();
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(nameFound);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();

  GDX.enableSensor(sensor); 
  GDX.start(periodMs);  //start sampling at the specified sample period in milliseconds

  //clear the display
  matrix.clear();
  delay(100);
}

void loop(){
  int points = 1;
  float time = 0;

  while (points <= numPointsToCollect){
    GDX.read();
    float sensorValue = GDX.getMeasurement(sensor);

    //print data to the LED Matrix
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(50);
    msg = String(sensorValue);
    char text[6];
    msg.toCharArray(text, 6);
    matrix.textFont(Font_5x7);
    matrix.beginText(4, 1, 0xFFFFFF);
    matrix.println(text);
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();

    time = time + periodSec;
    points ++;
  }
  
  GDX.stop();
  GDX.close();
  while(true); // done. Put Arduino into a do-nothing loop

}

