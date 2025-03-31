/*
In the example, a Go Direct sensor channel is configured for data 
collection at a specified rate and duration. The LED Matrix
on the UNO R4 Wifi is used to display the measurements.

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow: 

  1. There are 2 important variables to configure. Locate these in the code
  below and set them appropriately for your device and experimnent.
    a. Set the 'myDevice' variable with the order code and serial number of your device
    b. Set the 'sensor' variable with the sensor number to enable
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

 
// ****** IMPORTANT! INPUT YOUR DEVICE NAME ******
char* myDevice = "GDX-HD 151000C1"; // Replace with your Go Direct name and serial number. For example, myDevice = "GDX-HD 151000C1"
byte sensor = 1; // select the device's sensor to read. In most devices, the default sensor is 1


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
  String nameFound = GDX.getDeviceName();
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(nameFound);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();

  //make the sensor channel active
  GDX.enableSensor(sensor); 

  //start sampling at the specified sample period. Note this is in milliseconds
  GDX.start(2000);

  //clear the display
  matrix.clear();
  delay(100);
}

void loop(){
  int points = 1; //variable to keep track of how many pts have been collected
  
  while (points <= 10){
    GDX.read();
    float sensorValue = GDX.getMeasurement(sensor);

    //print data to the LED Matrix
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(50);
    String msg = String(sensorValue);
    char text[6];
    msg.toCharArray(text, 6);
    matrix.textFont(Font_5x7);
    matrix.beginText(4, 1, 0xFFFFFF);
    matrix.println(text);
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();

    points ++;
  }
  
  GDX.stop();
  GDX.close();
  while(true); // done. Put Arduino into a do-nothing loop

}

