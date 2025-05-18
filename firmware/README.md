# ESP32 Flight Controller

This project uses an **ESP32** microcontroller programmed via the **Arduino IDE**. It includes support for servos, iBus communication, and a web interface using asynchronous networking.

## 🛠️ Requirements

Before getting started, make sure the following components are installed in your Arduino IDE.

### 1. ESP32 Board Package

- **Platform**: ESP32 by Espressif Systems
- **Version**: `2.0.16`
- Install this via **Tools > Board > Boards Manager** by searching for `esp32`.

### 2. Required Libraries

Install the following libraries via **Sketch > Include Library > Manage Libraries**:

| Library           | Version | Author                            |
| ----------------- | ------- | --------------------------------- |
| ESP32Servo        | 3.0.6   | Kevin Harrington, John K. Bennett |
| IbusBM            | 1.1.4   | larrydfc                          |
| AsyncTCP          | 3.4.1   | ESP32Async                        |
| ESPAsyncWebServer | 3.1.0   | lacamera                          |

Use the version selector in the Library Manager to install the specified versions.

## ✅ Setup Instructions

1. Connect your ESP32 board to your computer via USB.
2. In Arduino IDE:
   - Select your board under **Tools > Board**.
   - Select the correct **Port** under **Tools > Port**.
3. Open the project `.ino` file.
4. Click **Upload** to flash the code to your ESP32.

## ℹ️ Notes

- Library versions are important for compatibility—be sure to match them exactly.
- If you're using an alternate development environment (like PlatformIO), set dependencies to the specified versions.
