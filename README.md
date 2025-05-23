# ESP32 Custom Drone

## 🚁 Overview

A compact, custom-built **5.5″ quadcopter drone** powered by an **ESP32 microcontroller**. This project includes both hardware design (3D-printed components, PCB layout) and firmware (Arduino-based) for flight control.

<img src="docs/DroneWithBattery.JPG" alt="Drone with battery" style="max-width: 600px; width: 100%;" />

---

## 🔧 Firmware

The flight control software is written in **C++** using the **Arduino framework**. It runs a **250Hz control loop** to maintain drone stability using the following key features:

- **Sensor Fusion**: A **Kalman Filter** fuses gyroscope and accelerometer data to estimate orientation.
- **Control System**: Stabilization is achieved using a **cascaded PID control loop**.
- **Remote Input**: Directional control is handled via a **radio receiver**.
- **Live PID Tuning**: The drone can host a **local Wi-Fi web interface** for real-time PID tuning.
  > _The web-based tuning interface was adapted from [pratikPhadte/ESP32-Flight-controller-](https://github.com/pratikPhadte/ESP32-Flight-controller-)._

<img src="docs/PID_Web_Page.png" alt="PID Web Page" style="max-width: 600px; width: 100%;" />

---

## 🛠️ Hardware Components

| Component             | Description                                                | Price   |
| --------------------- | ---------------------------------------------------------- | ------- |
| **Frame**             | FlyFishRC FIFTY5                                           | €50.00  |
| **Radio Receiver**    | Flysky FS-IA6B                                             | €20.00  |
| **Flight Controller** | ESP32-DevKitC, MP1584 (Buck Converter), MPU6050 (IMU), PCB | €20.00  |
| **Motors**            | 4x FlyfishRC Flash 2306 1750Kv                             | €60.00  |
| **ESC**               | 30.5mm 4-in-1 40A BLHeli32                                 | €50.00  |
| **Battery**           | 3S 1500mAh LiPo                                            | €20.00  |
| **Propellers**        | 4x 5.5″ Props                                              | €2.50   |
| **3D Printed Parts**  | Material cost for FC Case, RC Case, Feet                   | €2.00   |
| **Total**             |                                                            | €224,50 |

> Note: The listed prices are approximate and may vary depending on the retailer or manufacturer. In this build, we intentionally used some higher-quality (and more expensive) components to ensure that budget parts wouldn't become a limiting factor in performance. However, with careful sourcing and smart purchasing decisions, the overall cost of this drone can be significantly reduced.

---

## 🧠 Flight Controller

The flight controller (FC) is based on an **ESP32 DevKitC**, mounted on a custom PCB. It integrates:

- **MPU6050 IMU** for motion sensing
- **MP1584 buck converter** to step down 10V (from ESC) to 5V

Connections are made via **JST SH cables** to the ESC and radio receiver.

> ‼️ The VCC wire of the JST SH connector for the radio receiver is connected to the ESP32’s 3.3V output. However, the receiver used in this project (FS-IA6B) requires a minimum operating voltage of 4.0V. To address this, an additional VCC wire was manually soldered to the ESP32’s 5V pin to provide sufficient power. While some 5V receivers may appear to function at 3.3V in test setups, our testing showed that the receiver's output becomes unstable when the motors are running, likely due to temporary voltage drops. For reliable performance, ensure the receiver is properly powered with a stable 5V supply.

<img src="docs/DroneOpenFCCase.JPG" alt="Drone with open FC case" style="max-width: 600px; width: 100%;" />

---

## 🧾 PCB Design

Designed using **KiCad** and manufactured by **JLCPCB**.

### 🔌 Schematic

<img src="docs/Schematic.png" alt="Schematic" style="max-width: 600px; width: 100%;" />

### 🔄 Routing

<img src="docs/Routing.png" alt="Routing" style="max-width: 600px; width: 100%;" />

### 3D Model

<img src="docs/PCB_3D.png" alt="PCB 3D" style="max-width: 600px; width: 100%;" />

<img src="docs/PCB_3D_No_Models.png" alt="PCB 3D no models" style="max-width: 600px; width: 100%;" />

---

## 🖨️ 3D Printed Parts

All parts were designed in **Autodesk Fusion 360** and printed using **PLA**. **M2/M3 threaded heat inserts** are used to mount components to the carbon frame.

<img src="docs/All_3D_Printed_Parts.JPG" alt="All 3D Prints" style="max-width: 600px; width: 100%;" />

### 📦 Flight Controller Case

<img src="docs/FC_Case.png" alt="FC Case" style="max-width: 600px; width: 100%;" />
<img src="docs/FC_Case_Top.png" alt="FC Top" style="max-width: 600px; width: 100%;" />
<img src="docs/FC_Case_Bottom.png" alt="FC Bottom" style="max-width: 600px; width: 100%;" />
<img src="docs/FC_Bottom_Screws.JPG" alt="FC Bottom Screws" style="max-width: 600px; width: 100%;" />
<img src="docs/ESC.JPG" alt="ESC" style="max-width: 600px; width: 100%;" />

### 📡 Radio Receiver Case

<img src="docs/RC_Case.png" alt="RC Case" style="max-width: 600px; width: 100%;" />

<img src="docs/RC_Case_Open.JPG" alt="RC Case Open" style="max-width: 600px; width: 100%;" />

### 🧩 Miscellaneous Parts

**Spacers**  
<img src="docs/Spacers.JPG" alt="Spacers" style="max-width: 600px; width: 100%;" />

**Feet**  
Model from [Cults3D – FlyFishRC Volador Arm Protection](https://cults3d.com/de/modell-3d/spiel/flyfishrc-volador-5-6-full-arm-protection)  
<img src="docs/Foot.JPG" alt="Foot" style="max-width: 600px; width: 100%;" />

---

## 👥 Contributors

- Tilman Kurmayer (https://github.com/tchello45)
- Melina Leitermann (https://github.com/me1ina1)

## 📚 References & Inspirations

This project was inspired and guided by the following open-source resources:

- [CarbonAeronautics/Manual-Quadcopter-Drone](https://github.com/CarbonAeronautics/Manual-Quadcopter-Drone)
- [pratikPhadte/ESP32-Flight-controller-](https://github.com/pratikPhadte/ESP32-Flight-controller-)
