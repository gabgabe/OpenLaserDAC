# Changelog

## v0.2.0 — WiFi AP + Captive Portal + Status LED (2026-03-24)

Major rework of the network stack, new hardware drivers, and a critical memory fix that was preventing WiFi authentication.

### WiFi Authentication Fix (Root Cause)

The system had only **80 bytes of free internal DRAM** at runtime. WiFi needs ~200–300 bytes of DMA-capable internal memory to allocate management frame TX buffers for auth responses. With nothing left, every WPA2 handshake attempt failed with `wifi:m f auth` ("management frame auth failed").

**Fix — sdkconfig.defaults memory optimizations:**

| Setting | Before | After | Savings |
|---------|--------|-------|---------|
| Data cache | 64 KB | 32 KB | ~32 KB |
| WiFi static RX buffers | 16 | 6 | ~16 KB |
| WiFi dynamic RX buffers | 64 | 16 | reduced peak |
| WiFi static TX buffers | 16 | 6 | ~16 KB |
| TCP send buffer | 32,768 B | 5,744 B | ~27 KB/socket |
| Max sockets | 12 | 8 | minor |

Total freed: **~75 KB** of internal DRAM. ISR buffer + SPI data fits comfortably in 32 KB data cache; the ESP only sends small HTTP status pages, not bulk data.

---

### Network Architecture Redesign

Changed from **"Ethernet first, WiFi fallback"** to **"WiFi AP always on, Ethernet optional"**.

- **WiFi AP starts unconditionally** at boot — always available for configuration and monitoring.
- **W5500 Ethernet** is probed after WiFi is up. If present and linked, both interfaces run simultaneously.
- Ethernet failure paths now call `park_eth_gpios()` and return cleanly instead of falling through to `wifi_ap_start()`.

#### Boot Sequence (new)

```
pin-safe DAC GPIOs → status_led_init(RED) → vTaskDelay(2s)
→ nvs_flash_init() → park_eth_gpios()
→ wifi_ap_start() → dns_server_start()
→ probe W5500 (optional) → dac_timer_init()
→ start servers → dac_timer_start()
```

---

### New: Captive Portal DNS Server

**Files:** `src/web/dns_server.c`, `src/web/dns_server.h`

A lightweight DNS server (UDP port 53) that resolves **all** queries to `192.168.4.1`. When a phone connects to the AP, its captive portal detection immediately finds the web UI — no need to type the IP manually.

- Task runs at priority 5 on Core 0
- Stopped cleanly during OTA updates

### New: HTTP Captive Portal Redirects

All OS-specific captive detection endpoints (`/generate_204`, `/hotspot-detect.html`, `/connecttest.txt`, `/ncsi.txt`, `/library/test/success.html`) now return a **302 redirect to `http://192.168.4.1/`**, triggering the captive portal popup on Android, iOS, macOS, and Windows.

Replaces the previous approach of returning per-OS "correct" responses (204, "Success", "Microsoft NCSI") which did not trigger the popup.

---

### New: NeoPixel Status LED

**Files:** `src/hal/status_led.c`, `src/hal/status_led.h`

WS2812 RGB LED on GPIO 7 using the `espressif/led_strip` RMT driver.

| Color | Meaning |
|-------|---------|
| Red | Booting |
| Blue | WiFi AP only |
| Green | Ethernet (DHCP) |
| Magenta | Ethernet (static IP) |
| Green blink (300 ms) | Laser engaged |
| Orange | OTA update in progress |

Supports solid color (`status_led_set`) and FreeRTOS-timer-based blink (`status_led_blink` / `status_led_blink_stop`).

---

### DAC Boot Glitch Fix

DAC SPI pins (CS=GPIO 10, MOSI=GPIO 11, SCK=GPIO 12) are now driven to safe states **immediately** at `app_main()` entry, before any peripheral initialization. CS is pulled HIGH (deselected), MOSI and SCK are driven LOW.

Previously, during early boot, GPIOs default to high-impedance input mode. The DAC80508 CS would float LOW, causing spurious latch of random data on the RGB channels — visible as a full-white laser flash at power-on.

### Ethernet GPIO Parking

When W5500 hardware is absent, SPI2 pins (GPIO 13–16) are driven LOW and INT (GPIO 8) is configured as input with pull-up. These floating lines near the WiFi antenna were picking up RF noise.

`park_eth_gpios()` is called **before** `wifi_ap_start()` and again after any W5500 probe failure (since `spi_bus_initialize`/`spi_bus_free` touches the pins).

---

### Buffer-Empty Galvo Safety

On ISR underrun (no points in buffer), the DAC timer now **holds the galvo at the last known X/Y position** with laser blanked, instead of jumping to center (0, 0). This prevents a visible snap-to-center glitch between frames or during brief buffer starvation.

### W5500 Early Abort

When the W5500 is not responding (version and PHY registers both return 0x00), the driver now properly frees the SPI device and bus, and returns `ESP_ERR_NOT_FOUND` immediately. Previously it would continue initialization with a dead chip.

### Subnet-Aware UDP Broadcast

Ether Dream discovery broadcasts now bind to the active network interface (WiFi AP first, then Ethernet) and compute the subnet-specific broadcast address (`ip | ~netmask`) instead of using global `255.255.255.255`. Ensures discovery packets reach the correct subnet.

### OTA Safe Shutdown

Before writing firmware:
1. Stops DAC timer (laser safe)
2. Clears frame buffer
3. Disconnects Ether Dream protocol
4. Stops DNS server
5. Sets LED to orange

After sending the success response, stops HTTP server before rebooting.

---

### Heap Diagnostics

The PIPE status log now includes runtime memory stats:

```
PIPE| ... | heap=75000 int=42000 min=38000
```

- `heap` — total free heap (all regions)
- `int` — free internal DRAM (critical for WiFi)
- `min` — minimum free heap since boot (watermark)

Additional checkpoints logged at boot, pre-network, post-network, and post-DAC init stages.

---

### Files Changed

| File | Change |
|------|--------|
| `sdkconfig.defaults` | Memory optimization (WiFi auth fix) |
| `src/main.c` | Network redesign, boot glitch fix, GPIO parking, LED integration, heap diags |
| `src/hal/status_led.c` | **NEW** — NeoPixel driver |
| `src/hal/status_led.h` | **NEW** — NeoPixel API |
| `src/web/dns_server.c` | **NEW** — Captive portal DNS |
| `src/web/dns_server.h` | **NEW** — DNS server API |
| `src/hal/dac_timer.c` | Buffer-empty galvo hold |
| `src/hal/w5500_eth.c` | Early abort on missing chip |
| `src/input/etherdream_server.c` | Subnet-aware broadcast |
| `src/web/http_server.c` | Captive portal 302 redirects |
| `src/web/ota_handler.c` | Safe shutdown sequence |
| `src/CMakeLists.txt` | Added new source files + led_strip dep |
| `src/config.h` | Added `PIN_STATUS_LED` |
| `README.md` | Minor text fix |
