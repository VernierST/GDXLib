# Getting Started with Vernier Go Direct® Sensors and Arduino®

This guide describes how educators<sup>1</sup> can use [Vernier Go Direct Sensors](https://www.vernier.com/products/sensors/go-direct-sensors/) with Arduino microcontrollers and the GDXLib library. The GDXLib library makes communicating with the sensors of most Vernier Go Direct<sup>2</sup> devices possible, and easy, when working with an Arduino which supports the Arduino BLE library. The GDXLib library can communicate with up to seven sensors onboard one Go Direct device.

<sup>1</sup> Go Direct sensors are for educational use only. Vernier Science Education does not provide support for home use, industrial use, research, or medical.

<sup>2</sup> Go Direct spectrometers, Mini GC, Polarimeter, Go Wireless Heart Rate, and Cyclic Voltammetry System do not work with the GDXLib library. Go Direct sensors that are not supported, or that may require advanced programming, calibration, or analysis, include Blood Pressure, Sound, Ion-Selective Electrode, Optical Dissolved Oxygen, Conductivity, and timing/event devices like Photogates, Drop Counters, Projectile Launcher, Radiation, and Rotary Motion.

## Getting Started Requirements

- The Arduino microcontroller must be compatible with the ArduinoBLE library and have sufficient flash memory. This includes boards such as UNO R4 Wifi, Nano ESP32, Nano RP2040 Connect, and MKR Wifi 1010.
- The GDXLib library must be installed
- The ArduinoBLE library must be installed
- These two libraries must be included in your sketch

```
#include "ArduinoBLE.h"
#include "GDXLib.h"
```

## Sample Program

A simple example that uses a few of the GDXLib functions is shown below. In this example, a Go Direct Hand Dynamometer (GDX-HD 151000C1) is being used. The Hand Dynamometer has seven on-board sensors. The sensors are:

1. Force (N)
2. X-axis acceleration (m/s2)
3. Y-axis acceleration (m/s2)
4. Z-axis acceleration (m/s2)
5. X-axis gyro (rad/s)
6. Y-axis gyro (rad/s)
7. Z-axis gyro (rad/s)

In the example, the Force channel (1) is enabled and configured to collect 10 samples at a period of 1000ms. The data are printed to the Serial Monitor.

```
#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

void setup(){
  Serial.begin(9600);
  delay(6000);

  if (!GDX.open("GDX-HD 151000C1")) {
      Serial.println("GDX.open() failed. Disconnect/Reconnect USB");
      while (true);  //put the Arduino into a do-nothing loop
  }

  GDX.enableSensor(1); 
  
  GDX.start(1000);  // sample period in milliseconds
  for (int i = 0; i < 10; i++) {
    GDX.read();
    Serial.println(GDX.getMeasurement(1));
  }
  GDX.stop();
  GDX.close();
}

void loop(){
}
```

## Notes Regarding the GDXLib Functions 

The GDXLib functions for sensor data collection from a Go Direct device include:

- `GDX.open()`
- `GDX.getDeviceName()`
- `GDX.enableSensor()`
- `GDX.getSensorName()`
- `GDX.getUnits()`
- `GDX.start()`
- `GDX.read()`
- `GDX.getMeasurement()`
- `GDX.stop()`
- `GDX.close()`

Here is some more information about the functions:

### `GDX.open()`

- Initializes the Arduino's Bluetooth® Low Energy, and then begins a scan for the Go Direct device. When the device is found, the Go Direct device is connected to the Arduino as a peripheral.
- Returns true on success and false on failure
- This function has one parameter: `GDX.open(char* deviceName)`
- The deviceName parameter is a combination of the order code and serial number of the Go Direct device, such as `GDX.open("GDX-HD 151000C1”)`
- The deviceName parameter can also be set as "proximity" to open the nearest Go Direct device that has a threshold rssi signal stronger than -60, such as `GDX.open("proximity")`
- The `GDX.open()` function has no timeout. Therefore, it will pause the program indefinitely until it finds and connects the Go Direct device. If your code seems stuck at `GDX.open()` make sure the Go Direct device is turned on and the deviceName parameter is spelled correctly in the code. See the Troubleshooting section if you need more help.
- If you receive a failure with `GDX.open()`, disconnecting and reconnecting the Arduino USB cable usually resets the Arduino Bluetooth so that a second try will succeed. 
- If you power the Arduino off while it is in the middle of collecting data via Bluetooth from the Go Direct sensor, and then try to upload a new sketch, this may cause a failure at `GDX.open()`.

 ```
  if (!GDX.open("GDX-HD 151000C1")) {
      Serial.println("GDX.open() failed. Disconnect/Reconnect USB");
      while (true);  // if error, put the Arduino into a do-nothing loop
  }
```

### `GDX.getDeviceName()`

- Returns the Go Direct device name (order code and serial number) of the connected device
- No parameters in this function
- This function can provide good feedback that the device was found and connected.

```
if (!GDX.open("GDX-HD 151000C1")) {
      Serial.println("Problem starting GDX.open()"); 
      while (true); //put the Arduino into a do-nothing loop
  }

  // if open() was successful
  Serial.print("Found: ");
  Serial.println(GDX.getDeviceName());
  Serial.println();
```

### `GDX.enableSensor()`

- This function sets a specific sensor for data collection.
- This function has one parameter: `GDX.enableSensor(byte selectedSensor)`
- Each Go Direct device has one or more onboard sensors. These sensors have a unique sensor number. Use the sensor number to enable the sensor. Use this same sensor number as the argument for all of the other functions that have a selectedSensor parameter. 
- If, for example, you want to collect data from your Go Direct device's sensor #2
  - `GDX.enableSensor(2)`
- The function can also enable the device's default sensor by setting the selectedSensor as 255. Make sure to use 255 as the selectedSensor value for all of the other functions that require this parameter.
  - `GDX.enableSensor(255)`
- It will be important to know what sensors are on your device and what the sensor numbers are. A good list can be found at [Vernier Technical Information Library (TIL) #16315](https://www.vernier.com/til/16315)
- You must first open the device with `GDX.open()` before enabling the sensor(s) with `GDX.enableSensor()`
- Only after the sensor(s) have been enabled is it okay to start data collection by calling `GDX.start()`
- The GDXLib library allows for collecting data from up to seven sensors from one Go Direct device. 

```
// open() code goes before

// configure sensors 1, 2, and 3 for data collection
GDX.enableSensor(1);
GDX.enableSensor(2);
GDX.enableSensor(3);

// start() code goes after
```

### `GDX.getSensorName()`

- This function gets the sensor name of a specific sensor.
- This function has one parameter: `GDX.getSensorName(byte selectedSensor)`
- You cannot get the sensor name without first enabling the sensor with `GDX.enableSensor()`
- Each sensor on a Go Direct device has a unique identifier name that can be useful for display purposes, or as feedback. For example, the first three sensor names of the Hand Dynamometer are "Force", "X-axis acceleration", and "Y-axis acceleration"

```
// code to open()
// code to enableSensor()

//print headers using the sensor's name
Serial.print(GDX.getSensorName(1));
Serial.print(", ");
Serial.print(GDX.getSensorName(2));
Serial.print(", ");
Serial.println(GDX.getSensorName(3));
```

### `GDX.getUnits()`

- This function gets the units of a specific sensor.
- This function has one parameter: `GDX.getUnits(byte selectedSensor)`
- You cannot get the sensor units without first enabling the sensor with `GDX.enableSensor()`

```
// code to open()
// code to enableSensor()

//print headers by getting the enabled sensor's name and sensor units
  Serial.print(GDX.getSensorName(1));
  Serial.print(" ");
  Serial.println(GDX.getUnits(1));
```

### `GDX.start()`

- Start collecting data from all sensors that were selected in the `enableSensor()` function.
- This function has one parameter: `GDX.start(unsigned long period)`
- The period (time between samples) is set in milliseconds. 
- Do not set the period > 5000 ms, the `read()` functions has a 5 second timeout
- For example, `start(500)` has a period of 500 ms or 0.5 seconds, which is the same as 
having a sample rate of 2 samples/second. 

```
// code to open()
// code to enableSensor()

  GDX.start(1000);
```

### `GDX.read()`

- This function fills the buffer with data from the enabled sensors, but does not return the data
- The `getMeasurement()` function is used to return the data
- Place the function in a loop and make sure the loop can iterate fast enough to keep up with the sampling period (do not have other code in the loop, such as a delay, that might slow the loop).
- Note that currently the code drops data points during fast data collection. So if you are sampling at a period of 100 ms (10 samples/second) you might receive every data point. But if you are sampling at a period of 10 ms (100 samples/second) you might only retrieve one of every 3 data points. 
- There is a 5 second timeout, so do not set the period with `start()` > 5000 ms
- The `read()` function is blocking, meaning this function will pause the program as it waits for the data point to arrive from the sensor. This also means that you do not control the data collection speed by adding a delay to the read loop, the delay happens automatically, and accurately, in the read() function.

### `GDX.getMeasurement()`

- This function has one parameter: `GDX.getMeasurement(byte selectedSensor)`
- You cannot retrieve a measurement without first reading the data with `read()`
- This function is used to retrieve the data point for the specified sensor.
- Call `getMeasurement()` for each enabled sensor

```
// code to open()
// code to enable sensors 1, 2, and 3
// code to start()

void loop(){
  GDX.read();
  Serial.println(GDX.getMeasurement(1));
  Serial.println(GDX.getMeasurement(2));
  Serial.println(GDX.getMeasurement(3));
}
```

### `GDX.stop()`

- No parameters in this function
- Stops data collection
- If you want to restart data collection, you must call `start()` again

```
  GDX.start(2000);
  for(int row=1;row<5;row++){
    GDX.read();
    Serial.println(GDX.getMeasurement(1));
   }
   GDX.stop();  //stop data collection

  delay(5000);

  GDX.start(1000);  // restart data collection
  for(int row=1;row<5;row++){
    GDX.read();
    Serial.println(GDX.getMeasurement(1));
   }
   GDX.stop();  //stop data collection
```

### `GDX.close()`

- Disconnect the Go Direct device from Bluetooth
- Stop the Arduino's BLE
- If your sketch does not include `GDX.close()`, or you terminate the program prior to calling `GDX.close()`, the Arduino's Bluetooth may be left in a strange state. This may lead to a failure with `GDX.open() the next time you try to upload a new sketch. If this happens, disconnect and then reconnect the Arduino's USB cable. This helps to reset the Arduino Bluetooth.

## Troubleshooting

- In order to enable a specific sensor, you must know the sensor number. A list of sensor numbers can be found at [TIL 16315](https://www.vernier.com/til/16315)
- Double check that the Go Direct name and serial number are entered properly in the open() function.
- Turn on the Go Direct device after the Upload has finished and after opening the Serial Monitor (if the program prints to the Serial Monitor). This order of operations can help prevent any unintended connections with the device, such as occurs when the Arduino's sketch to be overwritten is trying to connect.
- Note that when you power on the Go Direct device a red Bluetooth LED will begin flashing. Once the Arduino pairs with the Go Direct device this LED will turn to green.
- Disconnect and then reconnect the Arduino's USB cable. This helps to reset the Arduino Bluetooth.
- It can always be a helpful troubleshooting step to confirm that you can collect data with your Go Direct device running Vernier's [Graphical Analysis App](https://www.vernier.com/downloads/graphical-analysis/)
- Make sure the battery power of the Go Direct device is good. This can be checked in Graphical Analysis
- If you cannot make a Bluetooth connection to the Go Direct device, try running the example found in Examples >> ArduinoBLE >> Central >> Scan. The ArduinoBLE library is used in the GDXLib library to make the Bluetooth connection. Running this example may help determine if the ArduinoBLE library is working properly with your Arduino board.
- Make sure your board's firmware and the ArduinoBLE library is up to date.
- If you are an educator, contact us at: support@vernier.com
- Post an issue in GitHub
- For Arduino coding questions, the Arduino forum is a good resource
