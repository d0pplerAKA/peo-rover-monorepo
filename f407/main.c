/**
 * @file  main.c
 * @brief Start the F407 rover firmware and coordinate host commands and manual driving.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Init order (architecture rule, see PEO_PINMAP.H):
 *   1. SystemInit()  - called by startup_stm32f40xx.s BEFORE main():
 *                      HSE 8 MHz -> PLL -> SYSCLK 168 MHz (HSE_VALUE must
 *                      be 8000000 in stm32f4xx_conf.h).
 *   2. RCC_Clock_Init()  - ALL peripheral clocks in one shot
 *                          (COMPONENTS/RCC; peripheral bus clock-enable masks).
 *   3. NVIC_InitAll()    - priority group + ALL IRQ channels in one shot
 *                          (COMPONENTS/NVIC).
 *   4. Component inits   - each module only configures its own peripheral
 *                          (GPIO/timers/USART/DMA/EXTI...), never RCC/NVIC.
 *   5. Sensors (VL53L1X/INA3221/MPU6050) are Init-ONLY here: start/stop is
 *      owned by the gamepad (Y/X/A toggle), NOT main. They configure EXTI
 *      channels, so they must not fire into later init code.
 *   6. If enabled, CONTROL_Init() comes AFTER sensor setup. Its 1 kHz
 *      interrupt immediately starts reading encoders and writing motor PWM.
 */
#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

#include "COMPONENTS/RCC/include/rcc.h"
#include "COMPONENTS/NVIC/include/nvic.h"
#include "COMPONENTS/DELAY/include/delay.h"
#include "COMPONENTS/USART/include/usart.h"
#include "COMPONENTS/I2C/include/i2c.h"
#include "COMPONENTS/TIMER/include/timer.h"
#include "COMPONENTS/MPU6050/include/mpu6050.h"
#include "COMPONENTS/CONTROL/include/control.h"
#include "COMPONENTS/ENCODER/include/encoder.h"
#include "COMPONENTS/MOTOR/include/motor.h"
#include "COMPONENTS/TOF400C/include/tof400c.h"
#include "COMPONENTS/BUILTIN_LED/builtin_led.h"
#include "COMPONENTS/TFT_SPI/include/tft_spi.h"
#include "COMPONENTS/TFT_UI/include/tft_ui.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include "COMPONENTS/INA3221/include/ina3221.h"
#include "COMPONENTS/ESP_LINK/include/esp_link.h"
#include "COMPONENTS/GAMEPAD/include/gamepad.h"

/* Dashboard refresh stamp: initialized after the boot screen is drawn so the
   screen shows startup info ~2 s before live telemetry takes over. */
static uint32_t s_last_tft_ms = 0;

/* How this firmware shares work:
 * main() performs startup and repeatedly handles commands. An interrupt handler
 * is a function the processor runs when hardware needs attention, temporarily
 * pausing less urgent code. TIM6 handles wheel feedback when CONTROL is enabled;
 * TIM14 reports telemetry at 50 Hz. Sensor sampling starts through gamepad toggles.
 * #if PEO_ENABLE_* selects code at build time; it is not a runtime on/off command.
 * The USART2 S3 link carries host commands/telemetry, USART3 C3 carries gamepad
 * snapshots, and USART1 is a separate human-readable debug console.
 */
/**
 * @brief Initialize the rover and repeatedly handle host commands and manual driving.
 *
 * Entered once by startup code after SystemInit(). Sensors are configured first; gamepad
 * buttons start sampling later. Hardware interrupts run the fast control and reporting
 * tasks while this loop handles commands.
 * @note No arguments are required.
 * @retval Never This function stays in its main loop and does not normally return.
 */
int main(void)
{
    /* 1-2. Total clock and interrupt initialization (components) */
    RCC_Clock_Init();
    NVIC_InitAll();

    /* 3. SysTick timestamps plus DWT delays (get_tick / mdelay for the motion driver). */
    TIMER_Init();

    /* 3b. Early TFT GPIO/SPI3 setup: the backlight PMOS gate (PD3) floats
           before the display driver owns it, and a floating gate can light
           the uninitialized panel (power-up stripes). Init() only configures
           pins/SPI and forces the backlight off - no delays. */
#if PEO_ENABLE_TFT
    TFT_SPI_Init();
#endif

    /* 4. Debug console */
    USART_DBG_Init(PEO_PM_DBG_BAUDRATE);     /* 115200, PA9/PA10 USB-TTL */

    /* 4b. On-board LED D2 (PA1) */
    BUILTIN_LED_Init();

    /* 4c. TFT panel startup (ST7789 sequence + DMA scan + backlight on;
           ~0.6 s total including the first-scan wait; GPIO/SPI3 were set up
           early in step 3b to kill the floating backlight). */
#if PEO_ENABLE_TFT
    TFT_SPI_Config();
#endif

    /* 5. Quadrature encoders (TIM1/3/4/8, 16-bit delta read) */
#if PEO_ENABLE_ENCODER
    ENCODER_Init();
#endif

    /* 6. Motor PWM (8 channels -> 4x DRV8871 IN1/IN2) */
#if PEO_ENABLE_MOTOR
    MOTOR_Init();
#endif

    /* 7. Sensors: Init ONLY - start/stop is owned by the gamepad toggle
          (Y = VL53 range, X = INA3221 current, A = MPU6050 DMP), not here. */
#if PEO_ENABLE_TOF400C
    TOF400C_Init();          /* 3x VL53L1X (EXTI0/1/2, I2C3 init implicit) */
#endif

#if PEO_ENABLE_INA3221
    INA3221_Init();          /* INA3221 current monitor (I2C2, PB10=SCL/PB11=SDA) */
#endif

#if PEO_ENABLE_MPU6050
    MPU6050_Init();
    MPU6050_Start();    /* TEMPORARY bench hook: no gamepad connected, so start the
                           DMP right away for dashboard verification. Remove once
                           the A-button toggle path is confirmed. */
#endif

    /* 8. Start the optional 1 kHz TIM6 controller after the hardware it uses.
          Its interrupt reads encoders and writes PWM immediately once enabled. */
#if PEO_ENABLE_CONTROL
    CONTROL_Init();
#endif

    /* 8b. ESP32-S3 link (USART2): send sensor-hub reports and receive host commands. */
    ESP_LINK_Init();
    ESP_LINK_StartReport();  /* TIM14 triggers sensor-hub reports; DMA moves the bytes to USART2. */

    /* 8c. Manual-control link (USART3): the Xbox controller reaches the F407 through the C3 serial bridge; the F407 parses input and arbitrates ownership. */
    GAMEPAD_Init();

    /* 9. Boot banner: diagnostics for clocks, build, settings, sensors, and enabled modules. */
    print_boot_banner();

    /* 9b. On-screen boot screen (title, version, build date, module states),
           kept visible ~2 s before the live dashboard takes over. */
#if PEO_ENABLE_TFT
    TFT_UI_ShowBoot();
    s_last_tft_ms = TIMER_GetTick() + 1500U;
#endif

    BUILTIN_LED_On();        /* Running indicator: turn on the LED after system startup completes. */

    while (1)
    {
#if PEO_ENABLE_TFT
        /* 10. Live dashboard refresh every 100 ms (10 Hz) so 100 Hz sensor
               data (MPU DMP) reads smoothly; framebuffer writes only - the
               circular DMA scan paints them within one 84 ms pass. */
        if ((int32_t)(TIMER_GetTick() - s_last_tft_ms) >= 100)
        {
            s_last_tft_ms = TIMER_GetTick();
            TFT_UI_ShowDashboard();
        }
#endif

        /* Handle the separate STOP request first, even when the ordinary command queue is full. */
        if (ESP_LINK_StopPending())
        {
            ESP_LINK_ClearStop();
            CONTROL_StopAll();
            SENSOR_SetCmdAck(ESP_CMD_STOP_ALL, ESP_CMD_OK);
        }

        /* Consume autonomous commands: the interrupt only queues them; the main loop executes them.
           Source: host computer -> WiFi -> ESP32-S3 -> USART2. */
        ESP_Cmd_t cmd;  /* Local copy of one queued host command, consumed by the main loop. */
        if (ESP_LINK_PopCommand(&cmd))
        {
            ESP_LINK_ExecCommand(&cmd);
        }

        /* Manual-control link: ownership arbitration and joystick mapping (C3 -> USART3, design A). */
        GAMEPAD_Process();

        mdelay(5);   /* Pause for 5 ms before the next main-loop pass; interrupt-driven tasks continue. */
    }
}
