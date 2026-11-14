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
 *      Sensors (I2C3/VL53L1X, MPU6050 DMP) come LAST: they start EXTI
 *      interrupts, so they must not fire into later init code.
 ******************************************************************************
 */
#include "stm32f4xx.h"
#include "PEO_PINMAP.H"

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

    /* 7. 1 kHz control loop (TIM6 - after encoders/motors, ISR reads them) */
#if PEO_ENABLE_CONTROL
    CONTROL_Init();
#endif

    /* 8-9. Sensors (LAST: they configure EXTI channels; interrupts are
       started later via MPU6050_Start() / TOF400C_StartRanging_Interrupt()) */
#if PEO_ENABLE_TOF400C
    TOF400C_Init();          /* 3x VL53L1X (EXTI0/1/2, I2C3 init implicit) */
#endif
#if PEO_ENABLE_INA3221
    INA3221_Init();          /* INA3221 current monitor (I2C2, PF1=SCL/PF0=SDA) */
    INA3221_PollInit();      /* TIM7 轮询读电流电压 -> sensorhub（SENSOR_RATE_INA_HZ） */
#endif
#if PEO_ENABLE_MPU6050
    MPU6050_Init();          /* MPU6050 DMP (PE3/EXTI3, start via MPU6050_Start()) */
#endif

    /* 8b. ESP32 link (USART2) - 帧收发：sensorhub 上报 + 手柄/命令接收 */
    ESP_LINK_Init();
    ESP_LINK_StartReport();  /* TIM14 定时器触发 sensorhub 上报（DMA 发送） */

    /* 9. Boot banner：全面的诊断信息（时钟/构建/参数/传感器/模块状态） */
    print_boot_banner();

    BUILTIN_LED_On();        /* 运行指示：系统启动完成，点亮 LED */

    while (1)
    {
        /* ── 部分1：Xbox 手柄遥控（周期读手柄状态 → 映射闭环目标速度）── */
        GamepadData_t pad;
        ESP_LINK_GetGamepad(&pad);
        if (ESP_LINK_IsGamepadConnected())
        {
            /* TODO 实测标定后填映射（摇杆中值 32768，满速 4424 = 0.01cm/s）：
             *   int16_t y = (int16_t)(pad.joy_ly - 32768);   // 前进/后退
             *   int16_t x = (int16_t)(pad.joy_lx - 32768);   // 转向
             *   // 死区、速度/转向系数、左右轮差速分配...
             *   // 调好后用 CONTROL_SetTargetSpeed(电机, 目标 0.01cm/s) 闭环驱动
             */
        }

        /* ── 部分2：上位机指令（命令队列消费：中断只入队，这里出队执行）── */
        ESP_Cmd_t cmd;
        if (ESP_LINK_PopCommand(&cmd))
        {
            ESP_LINK_ExecCommand(&cmd);
        }

        mdelay(10);   /* 10 ms 调度节拍 */
    }
}
