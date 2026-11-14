#ifndef __TOF400C_H__
#define __TOF400C_H__


#ifdef __cplusplus
extern "C" {
#endif

#include "COMPONENTS/I2C/include/i2c.h"
#include "COMPONENTS/VL53L1X/core/VL53L1X_api.h"


#define TOF400C_FRONT           0
#define TOF400C_LEFT            1
#define TOF400C_RIGHT           2

#define TOF400C_ON              1
#define TOF400C_OFF             0

/* ============================================================================
   VL53L1X 测距速率预设档位（DistanceMode + TimingBudget + InterMeasurement）
   ----------------------------------------------------------------------------
   单片连续测距模式，速率 = 1000 / max(TB, IMP) Hz（要求 IMP >= TB）。
   DM：1 = Short（上限 ~1.3 m，抗环境光强）；2 = Long（上限 ~4 m，默认）。
   TB 离散档：Short 15/20/33/50/100/200/500 ms；Long 33/50/100/200/500 ms。
   ----------------------------------------------------------------------------
   最大距离是「暗环境 + 白/灰目标」的理想值；强环境光下量程大幅缩短——
   Long 模式在 200 kcps/SPAD 强光下只剩 ~73 cm（官方 TB=100ms 实测）。
   ----------------------------------------------------------------------------
   | 档位 | DM    | TB(ms) | IMP(ms) | 速率  | 最大距离（暗环境）           |
   |------|-------|--------|---------|-------|-----------------------------|
   | FAST | Short |   20   |   20    | 50 Hz | ~1.3 m（Short 上限）        |
   | BAL  | Long  |   33   |   33    | 30 Hz | ~2 m（约，Long 最小 TB）    |
   | STD  | Long  |   50   |   75    | 13 Hz | ~2.7 m（约）                |
   | PREC | Long  |  100   |  100    | 10 Hz | ~3.6 m（官方 TB=100ms 实测）|
   | MAX  | Long  |  200   |  200    |  5 Hz | ~4.0 m（Long 满量程）       |
   ----------------------------------------------------------------------------
   参考（ST datasheet / UM2356）：Long 模式达到 4 m 需 TB≈140 ms；
   TB=100ms 实测 360 cm（暗）/ 73 cm（200kcps 强光）。20/33/50ms 的量程为插值估算。
   ============================================================================ */

#define PEO_PM_VL53_PROFILE_FAST   0   /* 50 Hz 快速避障 */
#define PEO_PM_VL53_PROFILE_BAL    1   /* 30 Hz 平衡     */
#define PEO_PM_VL53_PROFILE_STD    2   /* 13 Hz 标准     */
#define PEO_PM_VL53_PROFILE_PREC   3   /* 10 Hz 高精度   */
#define PEO_PM_VL53_PROFILE_MAX    4   /*  5 Hz 远距     */

/* 选择当前档位：改这一行（FAST/BAL/STD/PREC/MAX） */
#define PEO_PM_VL53_PROFILE   PEO_PM_VL53_PROFILE_STD

#if (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_FAST)
    #define PEO_PM_VL53_DISTANCE_MODE        1   /* Short */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     20
    #define PEO_PM_VL53_INTER_MEAS_MS        20
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_BAL)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     33
    #define PEO_PM_VL53_INTER_MEAS_MS        33
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_STD)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     50
    #define PEO_PM_VL53_INTER_MEAS_MS        75
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_PREC)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     100
    #define PEO_PM_VL53_INTER_MEAS_MS        100
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_MAX)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     200
    #define PEO_PM_VL53_INTER_MEAS_MS        200
#else
    #error "PEO_PM_VL53_PROFILE invalid: use FAST/BAL/STD/PREC/MAX"
#endif

extern VL53L1X_Result_t TOF400C_result[3];

void TOF400C_Init(void);
uint8_t TOF400C_BOOT_Init(uint8_t assigned_addr);   /* 0=成功, 非0=失败步骤 */
void TOF400C_StartRanging_Interrupt(uint8_t index);
void TOF400C_StopRanging_Interrupt(uint8_t index);


#ifdef __cplusplus
}
#endif


#endif
