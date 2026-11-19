#include "include/tof400c.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"


uint8_t TOF400C_ranging_flag[3] = {TOF400C_OFF, TOF400C_OFF, TOF400C_OFF};
const uint8_t TOF400C_target_addr[3] = {PEO_PM_VL53_FRONT_ADDR, PEO_PM_VL53_LEFT_ADDR, PEO_PM_VL53_RIGHT_ADDR};
VL53L1X_Result_t TOF400C_result[3];
/* 初始化状态：front/left/right，0=成功，非0=失败，3=未运行。 */
static uint8_t TOF400C_init_status[3] = {3, 3, 3};

void TOF400C_Init(void)
{
    I2C_TOF_Init();

    GPIO_InitTypeDef GPIO_S;
    GPIO_S.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_S.GPIO_OType = GPIO_OType_PP;
    GPIO_S.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_S.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_S.GPIO_Pin = PEO_PM_VL53_LEFT_XSHUT_PIN | PEO_PM_VL53_RIGHT_XSHUT_PIN | PEO_PM_VL53_FRONT_XSHUT_PIN;
    GPIO_Init(PEO_PM_VL53_XSHUT_PORT, &GPIO_S);

    GPIO_S.GPIO_Mode = GPIO_Mode_IN;
    GPIO_S.GPIO_OType = GPIO_OType_PP;
    GPIO_S.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_S.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_S.GPIO_Pin = PEO_PM_VL53_FRONT_INT_PIN | PEO_PM_VL53_LEFT_INT_PIN | PEO_PM_VL53_RIGHT_INT_PIN;
    GPIO_Init(PEO_PM_VL53_INT_PORT, &GPIO_S);

    SYSCFG_EXTILineConfig(PEO_PM_VL53_FRONT_INT_EXTI_PSRC, PEO_PM_VL53_FRONT_INT_EXTI_SRC);
    SYSCFG_EXTILineConfig(PEO_PM_VL53_LEFT_INT_EXTI_PSRC, PEO_PM_VL53_LEFT_INT_EXTI_SRC);
    SYSCFG_EXTILineConfig(PEO_PM_VL53_RIGHT_INT_EXTI_PSRC, PEO_PM_VL53_RIGHT_INT_EXTI_SRC);

    EXTI_InitTypeDef EXTI_TOF;
    EXTI_TOF.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_TOF.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_TOF.EXTI_LineCmd = ENABLE;
    EXTI_TOF.EXTI_Line = PEO_PM_VL53_FRONT_INT_EXTI_LINE;
    EXTI_Init(&EXTI_TOF);

    EXTI_TOF.EXTI_Line = PEO_PM_VL53_LEFT_INT_EXTI_LINE;
    EXTI_Init(&EXTI_TOF);

    EXTI_TOF.EXTI_Line = PEO_PM_VL53_RIGHT_INT_EXTI_LINE;
    EXTI_Init(&EXTI_TOF);

    EXTI_ClearITPendingBit(EXTI_Line0);
    EXTI_ClearITPendingBit(EXTI_Line1);
    EXTI_ClearITPendingBit(EXTI_Line2);

    
    GPIO_SetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_LEFT_XSHUT_PIN | PEO_PM_VL53_RIGHT_XSHUT_PIN | PEO_PM_VL53_FRONT_XSHUT_PIN);
    delay_ms(10);

    GPIO_ResetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_LEFT_XSHUT_PIN | PEO_PM_VL53_RIGHT_XSHUT_PIN | PEO_PM_VL53_FRONT_XSHUT_PIN);
    delay_ms(10);

    GPIO_SetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_RIGHT_XSHUT_PIN);
    delay_ms(10);
    if (TOF400C_BOOT_Init((uint8_t) PEO_PM_VL53_RIGHT_ADDR) != 0)
    {
        printf("[VL53] RIGHT init failed -> offline\n");
        GPIO_ResetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_RIGHT_XSHUT_PIN);
        TOF400C_init_status[TOF400C_RIGHT] = 1;
    }
    else
        TOF400C_init_status[TOF400C_RIGHT] = 0;

    GPIO_SetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_LEFT_XSHUT_PIN);
    delay_ms(10);
    if (TOF400C_BOOT_Init((uint8_t) PEO_PM_VL53_LEFT_ADDR) != 0)
    {
        printf("[VL53] LEFT init failed -> offline\n");
        GPIO_ResetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_LEFT_XSHUT_PIN);
        TOF400C_init_status[TOF400C_LEFT] = 1;
    }
    else
        TOF400C_init_status[TOF400C_LEFT] = 0;

    GPIO_SetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_FRONT_XSHUT_PIN);
    delay_ms(10);
    if (TOF400C_BOOT_Init((uint8_t) PEO_PM_VL53_FRONT_ADDR) != 0)
    {
        printf("[VL53] FRONT init failed -> offline\n");
        GPIO_ResetBits(PEO_PM_VL53_XSHUT_PORT, PEO_PM_VL53_FRONT_XSHUT_PIN);
        TOF400C_init_status[TOF400C_FRONT] = 1;
    }
    else
        TOF400C_init_status[TOF400C_FRONT] = 0;
}


uint8_t TOF400C_BOOT_Init(uint8_t assigned_addr)
{    
    int8_t status = 0;
    uint8_t sensor_state = 0;

    uint16_t counter = I2C_Timeout;
    while(sensor_state == 0)
    {
        if(counter == 0 || status != 0)
        {
            printf("[%x] Boot failed!\n", assigned_addr);
            return 1;
        }
        
        status = VL53L1X_BootState((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, &sensor_state);
        delay_ms(10);

        counter--;
    }

    delay_ms(5);
    status = VL53L1X_SensorInit((uint16_t) PEO_PM_VL53_DEFAULT_ADDR);
    if(status)
    {
        printf("[%x] sensor init failed!\n", assigned_addr);
        return 2;
    }

    delay_ms(5);
    status = VL53L1X_SetInterruptPolarity((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, 0);        // Active Low!!!
    if(status)
    {
       
        printf("[%x] sensor set irq polarity failed!\n", assigned_addr);
        return 3;
    }

    delay_ms(5);
    status = VL53L1X_SetROI((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, 4, 4);
    if(status)
    {
        printf("[%x] sensor set ROI failed!\n", assigned_addr);
        return 4;
    }

    delay_ms(5);
    status = VL53L1X_SetDistanceThreshold((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, 50, 3000, 3, 1);
    if(status)
    {
        printf("[%x] sensor set Threshold failed!\n", assigned_addr);
        return 5;
    }
    
    delay_ms(5);
    status = VL53L1X_SetDistanceMode((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, PEO_PM_VL53_DISTANCE_MODE);
    if(status)
    {
        printf("[%x] sensor set Ranging Mode failed!\n", assigned_addr);
        return 6;
    }

    /*
    *   TB - TimingBudget -> Sampling time
    *   Time cost in sampling.
    */
    delay_ms(5);
    status = VL53L1X_SetTimingBudgetInMs((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, PEO_PM_VL53_TIMING_BUDGET_MS);
    if(status)
    {
        printf("[%x] sensor set sampling time failed!\n", assigned_addr);
        return 7;
    }

    /*
    *   IMP - intermeasurement period in milliseconds - Sampling interval (TB is included in IMP)
    *   IMP >= TB
    */
    delay_ms(5);
    status = VL53L1X_SetInterMeasurementInMs((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, PEO_PM_VL53_INTER_MEAS_MS);
    if(status)
    {
        printf("[%x] sensor set Sampling interval failed!\n", assigned_addr);
        return 8;
    }

    delay_ms(5);
    status = VL53L1X_SetI2CAddress((uint16_t) PEO_PM_VL53_DEFAULT_ADDR, (uint8_t)(assigned_addr << 1));
    if(status)
    {
        printf("[%x] sensor set new addr failed!\n", assigned_addr);
        return 9;
    }

    delay_ms(5);
    status = VL53L1X_ClearInterrupt((uint16_t) assigned_addr);
    if(status)
    {
        printf("[%x] sensor clear interrupt flag failed!\n", assigned_addr);
        return 10;
    }

    return 0;
}


void TOF400C_StartRanging_Interrupt(uint8_t index)
{
    if (index >= 3)
        return;

    if(TOF400C_ranging_flag[index] == TOF400C_OFF)
    {
        TOF400C_ranging_flag[index] = TOF400C_ON;

        VL53L1X_StartRanging((uint16_t) TOF400C_target_addr[index]);
    }
}

void TOF400C_StopRanging_Interrupt(uint8_t index)
{
    TOF400C_ranging_flag[index] = TOF400C_OFF;
}

/** 读取某路传感器初始化状态：0=成功，非0=失败，3=未运行。 */
uint8_t TOF400C_GetInitStatus(uint8_t index)
{
    if (index >= 3)
        return 3;
    return TOF400C_init_status[index];
}

void EXTI0_IRQHandler(void) // Front
{
    int8_t status;

    if(EXTI_GetFlagStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        
        status = VL53L1X_GetResult((uint16_t) TOF400C_target_addr[TOF400C_FRONT], &TOF400C_result[TOF400C_FRONT]);
        if (status != 0)
            return;

        VL53L1X_ClearInterrupt((uint16_t) TOF400C_target_addr[TOF400C_FRONT]);

        SENSOR_PushDist(SENSOR_DIST_FRONT, TOF400C_result[TOF400C_FRONT].Distance, TOF400C_result[TOF400C_FRONT].Status);

        if(TOF400C_ranging_flag[TOF400C_FRONT] == TOF400C_OFF)
        {
            VL53L1X_StopRanging((uint16_t) TOF400C_target_addr[TOF400C_FRONT]);
        }
    }
}

void EXTI1_IRQHandler(void) // Left
{
    int8_t status;

    if(EXTI_GetFlagStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);
        
        status = VL53L1X_GetResult((uint16_t) TOF400C_target_addr[TOF400C_LEFT], &TOF400C_result[TOF400C_LEFT]);
        if (status != 0)
            return;

        VL53L1X_ClearInterrupt((uint16_t) TOF400C_target_addr[TOF400C_LEFT]);

        SENSOR_PushDist(SENSOR_DIST_LEFT, TOF400C_result[TOF400C_LEFT].Distance, TOF400C_result[TOF400C_LEFT].Status);

        if(TOF400C_ranging_flag[TOF400C_LEFT] == TOF400C_OFF)
        {
            VL53L1X_StopRanging((uint16_t) TOF400C_target_addr[TOF400C_LEFT]);
        }
    }
}

void EXTI2_IRQHandler(void) // Right
{
    int8_t status;

    if(EXTI_GetFlagStatus(EXTI_Line2) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line2);
        
        status = VL53L1X_GetResult((uint16_t) TOF400C_target_addr[TOF400C_RIGHT], &TOF400C_result[TOF400C_RIGHT]);
        if (status != 0)
            return;

        VL53L1X_ClearInterrupt((uint16_t) TOF400C_target_addr[TOF400C_RIGHT]);

        SENSOR_PushDist(SENSOR_DIST_RIGHT, TOF400C_result[TOF400C_RIGHT].Distance, TOF400C_result[TOF400C_RIGHT].Status);

        if(TOF400C_ranging_flag[TOF400C_RIGHT] == TOF400C_OFF)
        {
            VL53L1X_StopRanging((uint16_t) TOF400C_target_addr[TOF400C_RIGHT]);
        }
    }
}
