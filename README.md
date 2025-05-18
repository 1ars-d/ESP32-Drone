# ESP32 Custom Drone

## Overview

A compact, custom-built 5.5″ drone based on an ESP32 microcontroller. Includes hardware design (3D prints, PCB layout) and Arduino-based firmware for flight control.

![Drone with battery](docs/DroneWithBattery.JPG)

## Hardware

| Part              | Details                                               | Price  |
| ----------------- | ----------------------------------------------------- | ------ |
| Frame             | FlyFishRC FIFTY5                                      | 50,00€ |
| Radio Receiver    | Flysky FS-IA6B                                        | 20,00€ |
| Flight Controller | ESP32-DevKitC, MP1584 (Buck Converter), MPU6050 (IMU) | 30,00€ |
| Motors            | 4x FlyfishRC Flash 2306 1750Kv                        | 60,00€ |
| ESC               | 30.5mm 4in1 40A BLHeli32                              | 50,00€ |
| Battery           | 3s 1500mAh LiPo                                       | 20,00€ |
| Props             | 4x 5.5″ Props                                         | 2,50€  |
| 3D Printed parts  | Material cost for FC Case, RC Case, Feet              | 2,00€  |

## Flight Controller

The flight controller (FC) is an ESP32 Dev Kit C placed on a custom PCB. The PCB has an IMU (MPU6050) and a buck converter (MP1584) soldered to it. The buck converter regulates the 10V coming from the ESC down to 5V. ESC and Radio Receiver are connected through JST SH cables to the FC.

## Schematic

##
