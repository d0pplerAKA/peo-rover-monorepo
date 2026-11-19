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

## 4. asukiaaa XboxSeriesXControllerESP32 (vendored under components/xbox_controller/)

- **Purpose**: Xbox Series X controller BLE client library (HID reports,
  connection management, vibration).
- **License**: MIT.
- **Source**: asukiaaa (github.com/asukiaaa/arduino-XboxSeriesXControllerESP32).

---

Compliance note: the firmware statically links LGPL-2.1 code
(arduino-esp32). Rebuilding from this public source satisfies the
re-linking requirement. All vendored components retain their original
copyright headers.
