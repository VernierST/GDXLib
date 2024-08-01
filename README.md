# Getting Started with Vernier Go Direct® Sensors and Arduino®

This guide describes using [Vernier Go Direct Sensors](https://www.vernier.com/products/sensors/go-direct-sensors/) with Arduino microcontrollers and the GDXLib library. The GDXLib library makes reading sensor data from a Go Direct device possible, and easy, when working with an Arduino which supports the Arduino BLE library.

## Getting Started Requirements

- The GDXLib library must be installed
- The ArduinoBLE library must be installed
- The Arduino microcontroller must be compatible with the ArduinoBLE library and have sufficient flash memory. This includes boards such as UNO R4 Wifi, Nano ESP32, Nano RP2040 Connect, and MKR Wifi 1010.

## Sample Program

A simple example that uses a few of the GDXLib functions is shown below. In this example, a Go Direct Hand Dynamometer (GDX-HD 151000C1) is being used. The Hand Dynamometer has seven on-board sensors. The sensors are:

1. Force (N)
2. X-axis acceleration (m/s2)
3. Y-axis acceleration (m/s2)
4. Z-axis acceleration (m/s2)
5. X-axis gyro (rad/s)
6. Y-axis gyro (rad/s)
7. Z-axis gyro (rad/s)

In the example, the x-axis accleration channel (2) is enabled and configured to collect data at a period of 1000ms. The data are printed to the Serial Monitor.

```
#include "ArduinoBLE.h"
#include "GDXLib.h"
GDXLib GDX;

void setup(){
  Serial.begin(9600);
  delay(6000);

  if (!GDX.open("GDX-HD 151000C1")) {
      Serial.println("Problem starting GDX.open()"); 
      while (true);  // if error, put the Arduino into a do-nothing loop
  }

  GDX.enableSensor(2); 
  GDX.start(1000);
}

void loop(){
  GDX.read();
  Serial.println(GDX.getMeasurement(2));
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

### `gdx.vp_vernier_canvas()`

## Troubleshooting

- In order to enable a specific sensor, you must know the sensor number. A list of sensor numbers can be found at [TIL 16315](https://www.vernier.com/til/16315)
- Make sure your Go Direct device is turned on.
- Double check that the Go Direct name and serial number are entered properly in the open() function.
- It can always be a helpful troubleshooting step to confirm that you can collect data with your Go Direct device running Vernier's [Graphical Analysis App](https://www.vernier.com/downloads/graphical-analysis/)
- If you are having trouble making a Bluetooth connection to the Go Direct device, try running the example found in Examples >> ArduinoBLE >> Central >> Scan.
- Contact us at: support@vernier.com
