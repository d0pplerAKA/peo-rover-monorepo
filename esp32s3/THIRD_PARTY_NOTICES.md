# Third-Party Notices

This project includes the following third-party software components.
Each component retains its own license. The MIT license of this project
(`LICENSE`) covers only the original code written by the project author.

## 1. ESP-IDF (framework, not vendored)

- **Purpose**: Espressif IoT Development Framework (FreeRTOS, drivers,
  networking stack, build system).
- **License**: Apache License 2.0.
- **Source**: Espressif (github.com/espressif/esp-idf).
- **Key terms**: permissive; preserve copyright notices.

## 2. arduino-esp32 (as ESP-IDF component)

- **Purpose**: Arduino core for ESP32 used by the firmware (wiring APIs).
- **License**: GNU Lesser General Public License v2.1 (LGPL-2.1-only).
- **Source**: Espressif (github.com/espressif/arduino-esp32).
- **Key terms**: LGPL allows static linking into this firmware. Because the
  full firmware source is public in this repository, anyone can rebuild
  (re-link) the firmware with a modified LGPL library, which satisfies the
  LGPL re-linking requirement. See the build instructions in `README.md`.

## 3. esp32-camera (vendored under components/esp32-camera/)

- **Purpose**: ESP32 camera driver (OV2640 support, DMA frame buffers).
- **License**: Apache License 2.0.
- **Source**: Espressif (github.com/espressif/esp32-camera).
- **Key terms**: permissive; preserve copyright notices.

## 4. cJSON (vendored)

- **Purpose**: JSON parsing/generation used by cmd_parser and telemetry.
- **License**: MIT.
- **Source**: Dave Gamble (github.com/DaveGamble/cJSON).

## 5. Managed components (downloaded by IDF Component Manager)

- **esp-nimble-cpp** (h2zero) — Apache License 2.0.
- **esp-dsp**, **esp-modbus**, **esp-sr**, **esp_mqtt**, **rmaker_common**
  and other managed components — each under its own license declared in
  its component registry entry; these are not vendored in this repository
  and are fetched at build time.

---

Compliance note: the firmware statically links LGPL-2.1 code
(arduino-esp32). Rebuilding from this public source satisfies the
re-linking requirement. All vendored components retain their original
copyright headers.
