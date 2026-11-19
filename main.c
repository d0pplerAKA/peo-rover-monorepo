/**
 ******************************************************************************
 * @file    main.c
 * @brief   PEO Rover - STM32F407VGT6 firmware entry
 *
 * Init order (architecture rule, see PEO_PINMAP.H):
 *   1. SystemInit()  - called by startup_stm32f40xx.s BEFORE main():
 *                      HSE 8 MHz -> PLL -> SYSCLK 168 MHz (HSE_VALUE must
 *                      be 8000000 in stm32f4xx_conf.h).
 *   2. RCC_Clock_Init()  - ALL peripheral clocks in one shot
 *                          (COMPONENTS/RCC, includes ADC prescaler).
 *   3. NVIC_InitAll()    - priority group + ALL IRQ channels in one shot
 *                          (COMPONENTS/NVIC).
 *   4. Component inits   - each module only configures its own peripheral
 *                          (GPIO/timers/USART/DMA/EXTI...), never RCC/NVIC.
 *   5. Sensors (VL53L1X/INA3221/MPU6050) are Init-ONLY here: start/stop is
 *      owned by the gamepad (Y/X/A toggle), NOT main. They configure EXTI
 *      channels, so they must not fire into later init code.
 *   6. CONTROL_Init() comes AFTER sensors: the 1 kHz control loop is the
 *      only always-on loop besides sensors, so it starts last.
 ******************************************************************************
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
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include "COMPONENTS/INA3221/include/ina3221.h"
#include "COMPONENTS/ESP_LINK/include/esp_link.h"
#include "COMPONENTS/GAMEPAD/include/gamepad.h"

int main(void)
{
    /* 1-2. Total clock and interrupt initialization (components) */
    RCC_Clock_Init();
    NVIC_InitAll();

    /* 3. SysTick 1 ms timebase (get_tick / mdelay for the motion driver) */
    TIMER_Init();

    /* 4. Debug console */
    USART_DBG_Init(PEO_PM_DBG_BAUDRATE);     /* 115200, PA9/PA10 USB-TTL */

    /* 4b. On-board LED D2 (PA1) */
    BUILTIN_LED_Init();

    /* 5. Quadrature encoders (TIM1/3/4/8, 16-bit delta read) */
#if PEO_ENABLE_ENCODER
    ENCODER_Init();
#endif

    /* 6. Motor PWM (8 channels -> 4x DRV8871 IN1/IN2) */
#if PEO_ENABLE_MOTOR
    MOTOR_Init();
#endif

    /* 7. Sensors: Init ONLY — start/stop is owned by the gamepad toggle
          (Y = VL53 range, X = INA3221 current, A = MPU6050 DMP), not here. */
#if PEO_ENABLE_TOF400C
    TOF400C_Init();          /* 3x VL53L1X (EXTI0/1/2, I2C3 init implicit) */
#endif

#if PEO_ENABLE_INA3221
    INA3221_Init();          /* INA3221 current monitor (I2C2, PB10=SCL/PB11=SDA) */
#endif

#if PEO_ENABLE_MPU6050
    MPU6050_Init();
#endif

    /* 8. 1 kHz control loop (TIM6) — LAST: the only always-on loop besides
          sensors; its ISR reads encoders + writes PWM, so it starts after them. */
#if PEO_ENABLE_CONTROL
    CONTROL_Init();
#endif

    /* 8b. ESP32 link (USART2) - 帧收发：sensorhub 上报 + 命令接收 */
    ESP_LINK_Init();
    ESP_LINK_StartReport();  /* TIM14 定时器触发 sensorhub 上报（DMA 发送） */

    /* 8c. 手柄手操链路 (USART3) - Xbox 手柄经 C3 有线接入，F407 端解析 + 锁仲裁 */
    GAMEPAD_Init();

    /* 9. Boot banner：全面的诊断信息（时钟/构建/参数/传感器/模块状态） */
    print_boot_banner();

    BUILTIN_LED_On();        /* 运行指示：系统启动完成，点亮 LED */

    while (1)
    {
        /* 独立 STOP 请求优先处理，不受普通命令队列满影响。 */
        if (ESP_LINK_StopPending())
        {
            ESP_LINK_ClearStop();
            CONTROL_StopAll();
            SENSOR_SetCmdAck(ESP_CMD_STOP_ALL, ESP_CMD_OK);
        }

        /* ── 命令消费：中断只入队，这里出队执行（自主链路）──
           命令来源：上位机（WiFi -> ESP32 -> USART2）。 */
        ESP_Cmd_t cmd;
        if (ESP_LINK_PopCommand(&cmd))
        {
            ESP_LINK_ExecCommand(&cmd);
        }

        /* ── 手柄手操链路：锁仲裁 + 摇杆映射（C3 -> USART3，方案 A）── */
        GAMEPAD_Process();

        mdelay(5);   /* 10 ms 调度节拍 */
    }
}
