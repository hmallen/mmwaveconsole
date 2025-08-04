# mmwaveconsole

Browser-based interface for real-time human presence sensing and trajectory tracking powered by an ESP32 and the RD-03D 24 GHz FMCW radar module.

## Features
- Real-time presence detection (distance, speed, direction)
- Trajectory visualisation in any modern browser (WebSocket + HTML/JS)
- Self-hosted AP mode or integration into existing Wi-Fi network
- Configurable radar parameters (range, sensitivity, frame rate)
- Lightweight Arduino firmware (<512 kB) built with ESP-IDF / Arduino core

## Hardware
- ESP32-WROOM-32 DevKit (or compatible)
- RD-03D 24 GHz radar (UART interface)
- 5 V / 3.3 V power supply
- (Optional) USB-TTL adapter for debugging

## Getting Started

### Firmware
1. Install the latest Arduino IDE (>=2.3) with the ESP32 board package.
2. Clone this repository and open `mmwaveconsole/mmwaveconsole.ino`.
3. Select your ESP32 board and correct COM port.
4. Press the Upload button to flash.
5. Open Serial Monitor at **115200 baud** to see log output.

### Wiring
```text
RD-03D  | ESP32
--------|------
5 V     | 5 V
GND     | GND
TX      | GPIO16 (RX2)
RX      | GPIO17 (TX2)
```

### Web Interface
After the ESP32 boots it prints the IP address.  
Open that address in your browser to access the console.  
By default the device starts a Wi-Fi access point named **mmwaveconsole** (password **mmwave123**).  
You can change network credentials in `config.h`.

## Repository Structure
```text
.
├── mmwaveconsole/          # Arduino sketch & source files
│   ├── mmwaveconsole.ino   # Main firmware
│   └── ...
├── LICENSE
└── README.md
```

## Roadmap
- [ ] MQTT / Home Assistant integration  
- [ ] On-device configuration page  
- [ ] OTA firmware updates

## License
This project is licensed under the MIT License – see the `LICENSE` file for details.
