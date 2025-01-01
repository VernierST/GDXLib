/*
A Go Direct Photogate reads velocity (m/s) from sensor ch1. The velocity 
measurement is displayed on the UNO R4 Wifi LED Matrix.  

For information on Go Direct sensors, the GDXLib functions, and 
troubleshooting tips, see the Getting Started Guide at: 
https://github.com/VernierST/GDXLib

Steps to Follow: 

  1. Set the 'myDevice' variable with the name and serial number of your device
  2. Upload the program
  3.  When the upload is done, look for the Bluetooth icon on the LED Matrix, this 
  means it is searching for the Go Direct device.
  4. Turn on your Go Direct device
  5. When the LED Matrix scrolls "Ready", perform the experiment
  6. The velocity measurement will be displayed on the LED Matrix
  
*/

#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

// To use ArduinoGraphics APIs, the #include must come BEFORE ArduinoLEDMatrix
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;


// ****** IMPORTANT! CONFIGURE YOUR DEVICE ******
char* myDevice = "GDX-VPG 0J1005H2"; // For example, myDevice = "GDX-VPG 0J1005H2"
byte sensor = 1; // Photogate ch 1 = Velocity


float sensorValue; //variable to store the measurment
String msg; //variable to store the measurment as a string, in order to display on the LED Matrix
String nameFound;


void setup(){

  // show the bluetooth icon on the LED Matrix as it performs a bluetooth scan for the Go Direct device
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
  GDX.start(20); // sample period is not used in photogate measurements

  // scroll "ready" on the LED Matrix when it is time to use the photogate
  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(50);
  const char text2[] = "  READY!  ";
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(text2);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();
  // Turn off the display
  matrix.clear();
  delay(100);

  GDX.read(); // the program waits here until GDX.read() gets data from the photogate. 
  sensorValue = GDX.getMeasurement(sensor); // retrieve the measurement
  GDX.stop();
  GDX.close();
}

// Scroll the reading on the LED Matrix
void loop(){
  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(90);
  msg = String(sensorValue);
  char text3[6];
  msg.toCharArray(text3, 6);
  matrix.textFont(Font_4x6);
  matrix.beginText(4, 1, 0xFFFFFF);
  matrix.println(text3);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();
}
