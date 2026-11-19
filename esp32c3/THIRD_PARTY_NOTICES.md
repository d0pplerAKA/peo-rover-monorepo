# Third-Party Notices

This project includes the following third-party software components.
Each component retains its own license. The MIT license of this project
(`LICENSE`) covers only the original code written by the project author.

## 1. ESP-IDF (framework, not vendored)

- **Purpose**: Espressif IoT Development Framework (FreeRTOS, drivers,
  BLE stack, build system).
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

## 3. esp-nimble-cpp (managed component)

- **Purpose**: NimBLE C++ wrapper used by the BLE link to the Xbox gamepad.
- **License**: Apache License 2.0.
- **Source**: h2zero (github.com/h2zero/esp-nimble-cpp); fetched by the
  IDF Component Manager, not vendored.

## 4. asukiaaa Xbox controller libraries (vendored under components/xbox_controller/)

All files below are from Asuki Kono's MIT-licensed repositories, combined
into one local component:

- XboxSeriesXControllerESP32 (main BLE client) — MIT, Copyright (c) 2022 Asuki Kono
- XboxSeriesXHIDReportBuilder — MIT
- XboxControllerNotificationParser (+ Elite2 variant) — MIT

- **License**: MIT (full text in `components/xbox_controller/LICENSE`).
- **Source**: github.com/asukiaaa/arduino-XboxSeriesXControllerESP32 and
  related repositories.

---

Compliance note: the firmware statically links LGPL-2.1 code
(arduino-esp32). Rebuilding from this public source satisfies the
re-linking requirement. All vendored components retain their original
copyright headers.
