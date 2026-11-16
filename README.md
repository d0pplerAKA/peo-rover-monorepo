# PEO Rover — ESP32-S3 N8R8 bridge

ESP-IDF **v6.1** and Arduino-ESP32 **4.0.0-rc1** as a component. Gamepad BLE/HID parsing runs on a separate ESP32-C3; the S3 consumes UART state frames through `c3_link`. Keep `dependencies.lock` with the project.

## Hardware configuration

- Target: ESP32-S3 **N8R8** — 8 MB Quad Flash and 8 MB Octal PSRAM.
- Flash: DIO, 80 MHz. **Do not enable Octal Flash** for this module.
- PSRAM: Octal, 80 MHz, boot initialization and memory test enabled.
- Existing partition offsets are retained; this is not an NVS erase/migration.
- UART1 to STM32: TX GPIO11 → PD6, RX GPIO12 ← PD5, 921600 baud, common ground.
- Camera pins remain in `components/peo_pinmap/PEO_PINMAP.h`; GPIO35–37 are reserved for Octal PSRAM.
- The OV5640 module supplies its own XCLK. `pin_xclk` stays `GPIO_NUM_NC`; the configured **24 MHz** must match the actual camera oscillator. Verify the module marking before camera validation.

## Build on this Windows workstation

```powershell
Set-Location 'C:\Users\Danny\Documents\Espressif Files\peo_rover_esp32s3'
.\build.ps1
```

The script activates the EIM v6.1 environment and clears inherited MSYS variables. A successful command is not enough: check that the build emits `Project build complete` and updates `build\peo_rover_esp32s3.elf` and `.bin`.

## Flash and monitor

Disconnect motor power or lift the wheels for initial testing. Close other serial monitors first. Replace `COM11` if the ESP32 is on another port.

```powershell
Set-Location 'C:\Users\Danny\Documents\Espressif Files\peo_rover_esp32s3'
& 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'
idf.py -p COM11 flash monitor
```

Flash **bootloader + partition table + application**, not the application alone. `idf.py flash` uses the generated offsets. No eFuse changes or full-flash erase are required. Exit the monitor with Ctrl+].

Expected boot evidence:

1. PSRAM is detected and its startup memory test succeeds.
2. `peo_boot` reports `PSRAM=8388608 bytes`.
3. The C3 UART receiver and gamepad mapping tasks start; BLE initialization runs on the C3.
4. Network and task-assembly messages appear; camera failure must not reboot the bridge.
5. Repeat cold boots and test C3/gamepad reconnect, WiFi reconnect and simultaneous telemetry/video.

## Gamepad bridge on ESP32-C3

The C3 owns BLE/HID parsing and sends gamepad state frames to the S3. `c3_link_rx_task` validates the UART frames and publishes snapshots; `c3_map_task` maps those snapshots to motor commands. The S3 no longer initializes the gamepad BLE stack or requires its memory-retention constructor.

`tests/verify_firmware.py` checks the source and generated N8R8 configuration and reports the built ELF SHA256; it no longer checks a local BLE constructor.

## Local regression checks

The UART host suite targets the real `esp_link.cpp` and macro engine; the network suite targets network, parser, macro and telemetry logic using RTOS/HAL adapters. Neither suite simulates the ESP32 radio or camera hardware, and they do not cover the C3 BLE/HID implementation.

```powershell
python tests\run_link_tests.py
python tests\net_run_host_tests.py
& 'C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe' tests\verify_firmware.py
```

The host suite needs a C++17 `g++` on PATH (or set `CXX`). Temporary test executables are removed by the runner. Firmware checks require a completed ESP-IDF build.

## Safety boundary

The bridge sends STOP on startup and uses a latched, priority stop path independent of ordinary command queue capacity. Old-generation queued commands cannot rearm it. A stop may follow one previously committed UART frame, even if that frame has not yet physically started; this is not a hardwired emergency stop.

**If the ESP32 crashes, loses power, or the UART cable breaks, it cannot deliver STOP.** The STM32 must independently stop motors when commands become stale, and the vehicle needs an accessible physical power cutoff. This project does not modify the STM32 firmware.

## Control behavior after this repair

- `STOP_ALL`, connection loss and macro abort latch a stop and invalidate old queued work.
- A newly submitted explicit command or macro may resume after STOP has finished UART transmission. Old queued messages cannot rearm it.
- The gamepad must supply a **fresh neutral HID notification after STOP**, then a later off-center input to drive. Neutral idle does not continuously overwrite upper-computer commands.
- The S3 supervises valid C3 state frames with a 500 ms timeout; the mapping task also rejects snapshots older than 1 s. HID-input freshness is reported by the C3. Validate the controller's actual notification cadence before driving continuously.
- Normal macro completion waits until accepted commands finish UART transmission before its final STOP. This confirms transmission, **not STM32 execution/acknowledgment**.
- Use one active driving source at a time. A complete gamepad-vs-macro-vs-multi-client ownership policy is not implemented.
- Camera timing, wheel numbering/direction, joystick dead zone and full-scale speed still require hardware calibration.

## Recovery point

The original uncommitted source snapshot is retained locally as the Git tree reference `refs/repair-baselines/n8r8-before`; the matching crash ELF is retained as the blob reference `refs/repair-baselines/n8r8-crash-elf`. The working branch and original Git index were not replaced. These are local recovery references, not new commits or remote backups. Inspect a specific file before restoring it, and do not reset the whole working tree blindly.
