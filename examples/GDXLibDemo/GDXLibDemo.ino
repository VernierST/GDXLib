/*
 GDXLib Demo (v. 210211, using the 0.9.2 library code)
 This is a simple demo program for using GDX sensors on any Arduino 
 which supports the Arduino BLE library. This includes the Nano33 BLE,
 Arduino Nano33 Sense, and MKR WiFi 1010, and Arduino Uno WiFi Rev 2.
 */
#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;
  char strBuffer[32]="unassigned";
  char strUnits[32]="strUnits";
  int numberOfReadings=10;// change as you wish
  int readingNumber=0;// this will increment
  
void setup(){
  Serial.begin(9600);
  delay(500);
  Serial.println("Searching for");
  Serial.println("GDX Sensor");
  GDX.open();  //use this line for proximity pairing
      //or
  //GDX.open("GDX-ACC 0H1019K1",11, 1000);//or specify device, channel and period here 
      //or
  //GDX.open("GDX*ACC XXXXXXXX",7, 1000);//or specify device, channel and period here 
  Serial.print("Found: ");
  Serial.print (GDX.orderCode());
  Serial.print (" ");
  Serial.println (GDX.serialNumber());
  
  Serial.print("channelName: ");
  Serial.println (GDX.channelName());

  Serial.print("channelUnits: ");
  Serial.println (GDX.channelUnits());

  Serial.print("channelNumber: ");
  Serial.println (GDX.channelNumber());
  
  Serial.print("Battery status(%): ");
  Serial.println (GDX.batteryPercent());
  
  Serial.print("ChargeState: ");
  Serial.print (GDX.chargeState());
  Serial.println(" (0=idle, 1=charging, 2=charging complete, 3=error)");
  
  Serial.print("Bluetooth signal strength (RSSI): "); 
  Serial.println (GDX.RSSI());
  Serial.println ("threshold = -44 ");//!!!NOTE SET IN .CPP CODE
  
  Serial.print("samplePeriod: ");
  Serial.print(GDX.samplePeriodInMilliseconds());
  Serial.println(" milliseconds");
  
  GDX.start();
  delay(500);//let readings start
  
}//end of setup

void loop(){
  readingNumber++;// increment
  if (readingNumber<=numberOfReadings){
     Serial.print(readingNumber);
     Serial.print(" ");
     float channelReading =GDX.readSensor();
     Serial.print(channelReading);
     Serial.print(" ");
     Serial.println(GDX.channelUnits());
  }//end of if
   if (readingNumber==numberOfReadings+1){
      GDX.stop();
      GDX.close();
      Serial.println("Series of readings taken; stopping GDX data collection.");
  }//end of if
}
