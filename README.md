# WeMosD1R1_RetroverseBike_Encoders_GitHubRepo

GitHub repository of an Arduino project for Retroverse Bike Encoders with WeMos D1 R1 MCU board.

# Usage

## `./WeMosD1R1_RetroverseBike_Encoders`

- Open with [`Arduino IDE`](https://www.arduino.cc/en/software).
- Add the following **additional board manager urls**.

```text
http://arduino.esp8266.com/stable/package_esp8266com_index.json, https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

- Install the following **boards**.

```text
Arduino AVR Boards,
Arduino ESP32 Boards,
esp32,
esp8266
```

- Install the following **libraries**.

```text
ESP Rotary,
ESPSoftwareSerial
```

- Compile the project.
- Flash the project to any MCU boards compatible with **WeMos D1 R1**.

## `./Script-Utils/UDP_Packets_Receive.ps1`

- Run with [`PowerShell`](https://github.com/PowerShell/PowerShell)
- Receives UDP packets from **any remote IP and port** to **local port 50000**.

## `./Script-Utils/UDP_Packets_Send.ps1`

- Run with [`PowerShell`](https://github.com/PowerShell/PowerShell)
- Send UDP packets from **local port 50010** to **remote IP 192.168.31.240 port 50010**.

# Copyright

```
// Copyright (C) 2023-2025 Yucheng Liu. Under the GNU AGPL 3.0 license.
// GNU AGPL 3.0 License: https://www.gnu.org/licenses/agpl-3.0.txt .
```
