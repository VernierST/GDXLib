/*
In the example, 4 channels of the Go Direct Weather System are
configured (Wind Speed, Temperature, Relative Humidity, and Barometric
Pressure). The measurements are displayed on the UNO R4 Wifi LED Matrix. 

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
  4. When the upload is done, look for the Bluetooth icon on the LED 
    Matrix, this means it is searching for the Go Direct device.
  5. Turn on your Go Direct WTHR
  6. The sensor measurments will be displayed on the LED Matrix

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

// To use ArduinoGraphics APIs, the #include must come BEFORE ArduinoLEDMatrix
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;


// ****** IMPORTANT! INPUT YOUR DEVICE NAME ******
char* myDevice = "GDX-WTHR 13400106"; // Replace with your Go Direct name and serial number. For example, myDevice = "GDX-WTHR 13100004"

// variables to store the active channel numbers
byte sensor1 = 1; // Wind Speed is on sensor channel 1
byte sensor2 = 4; // Temperature is on sensor channel 4
byte sensor3 = 7; // Relative Humidity is on sensor channel  7
byte sensor4 = 10; // Barometric Pressure is on sensor channel 10

// variable to store each channel's name
String sensorName1;
String sensorName2;
String sensorName3;
String sensorName4;

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
  matrix.beginText(4, 1, 0xFFFFFF);
  matrix.println(nameFound);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();

  //configure all four channels to be active  
  GDX.enableSensor(sensor1); 
  GDX.enableSensor(sensor2); 
  GDX.enableSensor(sensor3); 
  GDX.enableSensor(sensor4); 

  sensorName1 = GDX.getSensorName(sensor1);
  sensorName2 = GDX.getSensorName(sensor2);
  sensorName3 = GDX.getSensorName(sensor3);
  sensorName4 = GDX.getSensorName(sensor4);

  // start sampling with a period of 20000 ms (20 seconds)  
  GDX.start(20000);  

  //clear the display
  matrix.clear();
  delay(100);
}

void loop(){
  int points = 1;

  while (points <= 5){
    GDX.read();
    float sensorValue1 = GDX.getMeasurement(sensor1);
    float sensorValue2 = GDX.getMeasurement(sensor2);
    float sensorValue3 = GDX.getMeasurement(sensor3);
    float sensorValue4 = GDX.getMeasurement(sensor4);

    scrollName(sensorName1);
    scrollValue(sensorValue1);
    scrollName(sensorName2);
    scrollValue(sensorValue2);
    scrollName(sensorName3);
    scrollValue(sensorValue3);
    scrollName(sensorName4);
    scrollValue(sensorValue4);

    points ++;
  }
  
  GDX.stop();
  GDX.close();
  while(true); // done. Put Arduino into a do-nothing loop
}

void scrollName(String sensorName){
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(50);
    matrix.textFont(Font_4x6);
    matrix.beginText(4, 1, 0xFFFFFF);
    matrix.println(sensorName);
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();
}  

void scrollValue(float sensorValue){
    //print data to the LED Matrix
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(50);
    String msg = String(sensorValue);
    char text[6];
    msg.toCharArray(text, 6);
    matrix.textFont(Font_4x6);
    matrix.beginText(4, 1, 0xFFFFFF);
    matrix.println(text);
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();
} 

