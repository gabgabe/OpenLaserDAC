<div align="center">

# ✨ OpenLaserDAC

**Open-source laser DAC firmware for ESP32-S3**

Turn your ESP32-S3 into a full-featured laser controller — plug in an Ethernet cable, fire up MadMapper or Liberation, and start drawing lasers in the air.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
![Status: Beta](https://img.shields.io/badge/Status-Beta%20🚧-orange)
![Platform: ESP32-S3](https://img.shields.io/badge/Platform-ESP32--S3-green)

</div>

<div align="center">

### ☕ Support the project


[![Buy Me a Coffee](https://img.shields.io/badge/Buy%20me%20a%20coffee-support-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=000000)](https://buymeacoffee.com/gabgabe)
[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/J3J31WDWSV)

</div>

> [!WARNING]
> **Here be lasers (and dragons) 🐉**
> This project is in active development — things might break, things might change, and your galvos might draw something unexpected. Use at your own risk, test on low power first, and remember: it's not a bug, it's a ✨ *creative feature* ✨.
> Contributions and bug reports are very welcome!

---

## 🎯 What is this?

OpenLaserDAC implements the **Ether Dream** protocol on an ESP32-S3, driving an 8-channel DAC80508 over SPI. It receives ILDA laser data via **Ethernet** (W5500) or **WiFi**, and it's compatible with software like **MadMapper**, **Liberation**, and any Ether Dream client.

Basically: cheap hardware, open firmware, real lasers. What could go wrong? *(see disclaimer above)*

---

## ⚡ Features

| | Feature | Details |
|---|---------|---------|
| 🌐 | **Ether Dream protocol** | TCP (port 7765) + UDP broadcast (port 7654), full state machine with flow control |
| 🔌 | **Dual network** | Ethernet (W5500) with DHCP, automatic WiFi AP fallback — it just works™ |
| 🚀 | **Fast DAC output** | GPTimer ISR on dedicated core, direct SPI writes (~1 µs per point) |
| 🎚️ | **Configurable scan rate** | 1–100 kHz (default 10 kHz) |
| 🔄 | **Lock-free ring buffer** | 8192-point SPSC buffer with atomic operations for cross-core sync |
| 🖥️ | **Web interface** | Status dashboard, configuration, and OTA firmware updates |
| 📡 | **OTA updates** | Upload firmware over the network — no more USB cables! |

---

## 🔧 Hardware

| Component | Description |
|-----------|-------------|
| **MCU** | ESP32-S3 (dual-core @ 240 MHz) |
| **DAC** | DAC80508 — 8-channel, 16-bit, SPI (SPI3, 50 MHz) |
| **Ethernet** | W5500 — SPI (SPI2, 50 MHz), interrupt-driven (GPIO 8) |

<details>
<summary>📌 Pin Mapping</summary>

| Function | GPIO |
|----------|------|
| DAC CS | 10 |
| DAC MOSI | 11 |
| DAC SCK | 12 |
| DAC MISO | 9 |
| ETH CS | 16 |
| ETH MOSI | 13 |
| ETH SCK | 14 |
| ETH MISO | 15 |
| ETH INT | 8 |

</details>

<details>
<summary>🎨 DAC Channel Assignment</summary>

| Channel | Signal |
|---------|--------|
| OUT3 | 🔵 Blue |
| OUT4 | 🟢 Green |
| OUT5 | 🔴 Red |
| OUT6 | ↔️ X |
| OUT7 | ↕️ Y |

</details>

---

## 🏗️ Architecture

The firmware squeezes every drop of performance out of both ESP32-S3 cores:

| Core | Role |
|------|------|
| **Core 0** | Network stack, Ether Dream server (TCP/UDP), HTTP server, WiFi AP |
| **Core 1** | GPTimer ISR (per-point DAC output at scan rate), buffer refill task |

Data flows through a lock-free pipeline:

```
Ether Dream TCP → frame_buffer (8192 pts, SPSC ring) → ISR sub-buffer (1024 pts) → DAC80508
```

The ISR reads from a local sub-buffer for deterministic timing. A refill task on Core 1 replenishes the sub-buffer from the main ring buffer using atomic load/store operations. Zero locks, zero drama. 🎭

---

## 🌍 Network Modes

The firmware is smart about connectivity — it tries these in order:

| Priority | Mode | Details |
|----------|------|---------|
| 1️⃣ | **Ethernet DHCP** | Waits 10s for a DHCP lease via W5500 |
| 2️⃣ | **Static IP** | Falls back to `192.168.77.1` if DHCP times out |
| 3️⃣ | **WiFi AP** | If no Ethernet → creates its own network |

WiFi AP settings:
- **SSID:** `OpenLaserDAC`
- **Password:** `openlaser`
- **IP:** `192.168.4.1`

---

## 🚀 Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (recommended via VS Code extension)
- ESP-IDF framework (automatically managed by PlatformIO)

### Build & Flash

```bash
# Build the firmware
pio run

# Upload via USB (first time)
pio run --target upload

# Serial monitor — see what's happening
pio device monitor
```

### OTA Update

After the first USB flash, ditch the cable! Update over the network:

```bash
curl -X POST -F "file=@.pio/build/openlaserdac/firmware.bin" http://<DEVICE_IP>/api/ota
```

---

## 🖥️ Web Interface

Open your device's IP in a browser to access the dashboard:

- 📶 Connection status (Ethernet / WiFi)
- 🎯 Ether Dream client state
- 📊 Buffer level and scan rate
- 📦 OTA firmware upload

---

## ⚙️ Configuration

Tweak the defaults in `src/config.h`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SCAN_RATE_DEFAULT_HZ` | 10000 | Default output scan rate |
| `SCAN_RATE_MIN_HZ` | 1000 | Minimum scan rate |
| `SCAN_RATE_MAX_HZ` | 100000 | Maximum scan rate |
| `FRAME_BUFFER_SIZE` | 8192 | Ring buffer capacity (points) |
| `WIFI_AP_SSID` | OpenLaserDAC | WiFi AP name |
| `WIFI_AP_PASS` | openlaser | WiFi AP password |
| `ETH_DHCP_TIMEOUT_MS` | 10000 | DHCP timeout before static fallback |

---

## 🗺️ Roadmap

Here's where OpenLaserDAC is heading — this is a passion project so timelines are "when it's done" 😄

- [ ] 🎛️ **Multi-DAC support** — Compatibility with more DAC chips beyond the DAC80508 (MCP4922, AD5754, etc.)
- [ ] 🌐 **IDN protocol** — Support for the [ILDA Digital Network (IDN)](https://www.ilda.com/resources/StandardsDocs/ILDA_IDN-Stream_rev001.pdf) protocol alongside Ether Dream
- [ ] 📡 **More input protocols** — LaserOS, custom UDP streams, and other community-requested protocols
- [ ] 🖥️ **Better web UI** — Real-time point visualization, drag-and-drop config, dark mode, mobile-friendly dashboard
- [ ] 🎨 **Color calibration** — Per-channel gain/offset tuning from the web interface
- [ ] 📐 **Geometric correction** — Keystone, rotation, and scaling adjustable on-device
- [ ] 🔌 **Hardware abstraction** — Cleaner HAL layer so porting to other MCUs becomes feasible
- [ ] 🧪 **Test patterns** — Built-in test frames (ILDA test pattern, grid, circle) for alignment without external software
- [ ] 📖 **Documentation & guides** — Build guides, BOM, PCB designs, and video tutorials

Got an idea? [Open an issue](../../issues) — all suggestions welcome!

---

## 📁 Project Structure

```
├── CMakeLists.txt          # ESP-IDF project definition
├── platformio.ini          # PlatformIO build configuration
├── partitions.csv          # Flash partition table (4 MB)
├── sdkconfig.defaults      # ESP-IDF SDK configuration
└── src/
    ├── config.h            # Pin mapping, constants, types
    ├── main.c              # App entry, network init, status loop
    ├── core/
    │   └── frame_buffer    # Lock-free SPSC ring buffer
    ├── hal/
    │   ├── dac80508        # DAC driver (SPI, IRAM ISR-safe)
    │   ├── dac_timer       # GPTimer ISR + refill task
    │   └── w5500_eth       # Ethernet driver (SPI + interrupt)
    ├── input/
    │   └── etherdream_server  # Ether Dream protocol (TCP + UDP)
    └── web/
        ├── http_server     # Status dashboard + API
        └── ota_handler     # OTA firmware upload
```

---

## 📄 License

This project is licensed under the **GNU General Public License v3.0** — see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**Made with ❤️ and mass amounts of ☕ — if your laser draws a straight line, it's working!**

*If it draws something else... well, abstract art is valid too.*

</div>
