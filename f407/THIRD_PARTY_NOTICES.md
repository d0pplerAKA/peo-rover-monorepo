# Third-Party Notices

This project includes the following third-party software components.
Each component retains its own license. The MIT license of this project
(`LICENSE`) covers only the original code written by the project author.

## 1. ST Standard Peripheral Library (FWLIB/)

- **Purpose**: STM32F4 peripheral drivers (GPIO/TIM/USART/DMA/SPI/I2C...).
- **License**: ST Ultimate Liberty Software License Agreement (proprietary).
- **Source**: STMicroelectronics, distributed with Keil MDK.
- **Key terms**: licensed for use solely with ST microcontrollers; any
  redistribution (source or binary, modified or not) must retain all
  copyright notices and the license text. This library is NOT MIT-licensed.

## 2. ARM CMSIS (CMSIS/)

- **Purpose**: Cortex-M core support headers, startup and system files.
- **License**: Apache License 2.0 (CMSIS core); some components BSD-3-Clause.
- **Source**: ARM (github.com/ARM-software/CMSIS_5).
- **Key terms**: permissive; preserve copyright notices.

## 3. VL53L1X Ultra Lite Driver (COMPONENTS/VL53L1X/)

- **Purpose**: ST VL53L1X time-of-flight ranging sensor API (STSW-IMG009).
- **License**: ST SLA0103 Software License Agreement (proprietary).
- **Source**: STMicroelectronics (st.com/en/embedded-software/stsw-img009.html).
- **Key terms**: licensed for use with ST VL53L1X devices; redistribution
  must retain copyright notices and the license text. This library is
  NOT MIT-licensed.

## 4. InvenSense Embedded Motion Driver 6.12 (COMPONENTS/motion_driver_dmp/)

- **Purpose**: MPU6050 DMP (Digital Motion Processor) firmware interface
  and motion algorithms.
- **License**: InvenSense License Agreement for Embedded MotionDrivers
  (proprietary, non-standard).
- **Source**: TDK InvenSense (invensense.com/developers).
- **Key terms**: licensed for use solely and exclusively with InvenSense
  products (MPU6050); redistribution is permitted for the purpose of
  integration into non-GPL open-source software; copyright notices must
  be preserved. This library is NOT MIT-licensed.

---

Compliance note: all proprietary components above are used with their
respective vendor hardware (STM32 MCU, VL53L1X, MPU6050) and retain their
original copyright headers and license texts in the source tree.
