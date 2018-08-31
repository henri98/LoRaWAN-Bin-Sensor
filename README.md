

# LoRaWAN Bin Sensor

## Introduction

Bin sensor based on arduino pro mini transmitting data using LoRaWAN over a RFM95W transceiver module. The small VL53L0X time-of-flight ranging sensor is used to measure the distance to the waste. A 3.6V 18650 battery is used to power the sensor. 

![](.\img\1535716462895.png)

![](.\img\scheme.png)

## Hardware

See scheme above, scheme is based on Arduino Pro Mini. To reduce power consumption, the leds and the voltage regulator are removed. The battery should be connected to VCC and GND

Future Improvements:
Add battery circuit?

## Software

Sketch can be found in the src folder.

This is a PlatformIO project, so Atom or Visual Studio Code can be used to programm the 328p

The folowing libarys are used:

- LoraMAC-in-C for Arduino thank to Thomas Telkamp and Matthijs Kooijman (https://github.com/matthijskooijman/arduino-lmic)
- Low-Power: Lightweight low power library for Arduino (https://github.com/rocketscream/Low-Power)
- VL53L0X library for Arduino (https://github.com/pololu/vl53l0x-arduino)

Future Improvements:
Disable brown out detection.


![](.\img\1535716333198.png)

![](.\img\1535716340736.png)

![](.\img\1535716366459.png)