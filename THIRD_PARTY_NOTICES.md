# Third-Party Notices

This monorepo bundles four projects. Each project's own code is MIT
(Copyright (c) 2026 Danny); every third-party component retains its own
license. Per-project details:

| Project | Third-party licenses involved | Detail |
|---|---|---|
| `f407/` | ST Liberty License (StdPeriph), Apache-2.0/BSD (CMSIS), ST SLA0103 (VL53L1X ULD), InvenSense proprietary (Motion Driver) | `f407/THIRD_PARTY_NOTICES.md` |
| `esp32s3/` | Apache-2.0 (ESP-IDF, esp32-camera), **LGPL-2.1** (arduino-esp32), MIT (cJSON), managed components under their own licenses | `esp32s3/THIRD_PARTY_NOTICES.md` |
| `esp32c3/` | Apache-2.0 (ESP-IDF, esp-nimble-cpp), **LGPL-2.1** (arduino-esp32), MIT (asukiaaa Xbox controller library) | `esp32c3/THIRD_PARTY_NOTICES.md` |
| `upper_computer/` | LGPL v3 (PySide6), BSD-3-Clause (numpy), Apache-2.0 (opencv-python) | `upper_computer/THIRD_PARTY_NOTICES.md` |

## Key compliance notes

- The MIT license covers only the author's original code.
- ST (StdPeriph, VL53L1X ULD) and InvenSense components are proprietary
  licenses restricted to their own hardware; they are used with that
  hardware and all copyright notices are preserved.
- The S3/C3 firmware statically links LGPL-2.1 code (arduino-esp32);
  rebuilding the firmware from this public source satisfies the LGPL
  re-linking requirement.
- PySide6 (LGPL v3) is imported dynamically by Python; users can replace
  the PySide6 version at any time.
