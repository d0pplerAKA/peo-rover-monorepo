/**
 * @file  tft_ui.c
 * @brief On-screen views for the TFT_SPI display (boot screen, dashboard).
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * BOOT SCREEN layout (320x172, observer's top-left origin; 5x7 font):
 *
 *   y=6   PEO ROVER            scale 3, white (title)
 *   y=34  FW 28                scale 2, cyan  (protocol version, raw value)
 *   y=48  <__DATE__>           scale 2, cyan  (build date)
 *   y=66  MOTOR : OFF          scale 2, white (module build switches)
 *   y=80  CTRL  : OFF
 *   y=94  ENC   : OFF
 *   y=108 TOF   : OFF
 *   y=122 INA   : OFF
 *   y=136 MPU   : OFF
 *   y=154 BOOT OK              scale 2, green
 *
 * DASHBOARD layout (user-directed: power on top, speed+attitude middle,
 * distances bottom; air between the text rows):
 *
 *   y=4    V 11.23V             I 1.24A      scale 3
 *   y=34   M1  123   M2  -98    cm/s         scale 2 (annotation scale 1)
 *   y=51   M3  101   M4  115
 *   y=68   P  12  R  -3  Y  178 deg
 *   y=92   FRONT 1234mm
 *   y=110  LEFT  1234mm
 *   y=128  RIGHT 1234mm
 *   y=148  UP 123.45 s                    scale 1 (footer, green)
 *
 * Values are right-aligned inside fixed-width fields so their unit labels
 * (V/A/mm/deg/...) sit immediately after the field edge and never drift.
 *
 * All y values keep 3-4 px of air between rows; the footer ends at 155 <= 172,
 * leaving a small band at the bottom for future status lines.
 *
 * REFRESH STRATEGY (non-blocking, main-loop friendly): static labels are
 * drawn once on the first dashboard paint (which also clears the boot
 * screen); after that every 100 ms tick only clears+redraws the value
 * fields whose content actually changed. No delay call anywhere; a full
 * tick costs ~0.1-0.5 ms of framebuffer writes, leaving the command loop
 * untouched.
 */
#include "include/tft_ui.h"
#include "COMPONENTS/TFT_SPI/include/tft_spi.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include "COMPONENTS/TIMER/include/timer.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============================================================================
   SHARED HELPERS
   ============================================================================ */

/**
 * @brief Draw one "NAME : ON/OFF" module line at the given row (boot screen).
 *
 * @param y    Top pixel row of the line.
 * @param name Module label, max 5 characters for column alignment.
 * @param on   1 prints ON, 0 prints OFF.
 */
static void TFT_UI_DrawModuleLine(uint16_t y, const char *name, uint8_t on)
{
    char line[16];

    sprintf(line, "%-5s : %s", name, on ? "ON " : "OFF");
    TFT_SPI_DrawString(8, y, line, TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
}

/**
 * @brief Convert the MPU quaternion to Euler angles in degrees (ZYX order).
 *
 * DMP quaternion (w,x,y,z). Microlib's double math is software-emulated, but
 * this runs once per dashboard refresh (2 Hz) - cost is negligible.
 * @param w,x,y,z Quaternion components (normalized by the DMP).
 * @param pitch   Output pitch in degrees (-90..90).
 * @param roll    Output roll in degrees (-180..180).
 * @param yaw     Output yaw in degrees (-180..180).
 */
static void TFT_UI_QuatToEulerDeg(float w, float x, float y, float z,
                                  int16_t *pitch, int16_t *roll, int16_t *yaw)
{
    double sinp;
    double p;
    double r;
    double yw;

    sinp = 2.0 * ((double)w * (double)y - (double)z * (double)x);
    if (sinp > 1.0) { sinp = 1.0; }
    if (sinp < -1.0) { sinp = -1.0; }
    p = asin(sinp);

    r = atan2(2.0 * ((double)w * (double)x + (double)y * (double)z),
              1.0 - 2.0 * ((double)x * (double)x + (double)y * (double)y));

    yw = atan2(2.0 * ((double)w * (double)z + (double)x * (double)y),
               1.0 - 2.0 * ((double)y * (double)y + (double)z * (double)z));

    *pitch = (int16_t)(p * 57.29577951308232);
    *roll  = (int16_t)(r * 57.29577951308232);
    *yaw   = (int16_t)(yw * 57.29577951308232);
}

/* ============================================================================
   BOOT SCREEN
   ============================================================================ */

/**
 * @brief Draw the boot screen.
 */
void TFT_UI_ShowBoot(void)
{
    /* Title: PEO ROVER at 3x scale. */
    TFT_SPI_DrawString(16, 6, "PEO ROVER", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 3);

    /* Firmware: the raw protocol version (PEO_FW_VERSION) so the screen and
       the SensorData.fw_version field always agree. */
    TFT_SPI_DrawNumber(8, 34, PEO_FW_VERSION, TFT_LCD_COLOR_CYAN, TFT_LCD_COLOR_BLACK, 2);

    /* Build date from the compiler (e.g. "Nov 20 2026"). */
    TFT_SPI_DrawString(8, 48, __DATE__, TFT_LCD_COLOR_CYAN, TFT_LCD_COLOR_BLACK, 2);

    /* Module build switches, mirroring the serial boot banner. */
    TFT_UI_DrawModuleLine(66,  "MOTOR", PEO_ENABLE_MOTOR);
    TFT_UI_DrawModuleLine(80,  "CTRL",  PEO_ENABLE_CONTROL);
    TFT_UI_DrawModuleLine(94,  "ENC",   PEO_ENABLE_ENCODER);
    TFT_UI_DrawModuleLine(108, "TOF",   PEO_ENABLE_TOF400C);
    TFT_UI_DrawModuleLine(122, "INA",   PEO_ENABLE_INA3221);
    TFT_UI_DrawModuleLine(136, "MPU",   PEO_ENABLE_MPU6050);

    /* Green success line: firmware reached the end of startup. */
    TFT_SPI_DrawString(8, 154, "BOOT OK", TFT_LCD_COLOR_GREEN, TFT_LCD_COLOR_BLACK, 2);
}

/* ============================================================================
   DASHBOARD
   ============================================================================ */

/* Dirty-region caches: the last value painted in each field. Sentinels force
   a full paint on the first tick after the static labels are drawn. */
static int32_t  s_lv_volt100 = INT32_MIN;              /* V x 100 */
static int32_t  s_lv_cur100  = INT32_MIN;              /* A x 100 */
static int32_t  s_lv_speed[4] = { INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN };  /* cm/s */
static int16_t  s_lv_pitch   = INT16_MIN;              /* deg */
static int16_t  s_lv_roll    = INT16_MIN;              /* deg */
static int16_t  s_lv_yaw     = INT16_MIN;              /* deg */
static uint16_t s_lv_dist[3] = { 0xFFFFU, 0xFFFFU, 0xFFFFU };  /* mm (VL53 max 4000) */
static int32_t  s_lv_up_s100 = INT32_MIN;             /* uptime seconds x 100 */
static uint8_t  s_dash_static_drawn = 0;               /* Static labels painted once. */

/* Value-field clear boxes: sized for the widest possible text of each field,
   so a clear+redraw never leaves stale glyph pixels behind. */
#define UI_FIELD_V_X     (8)
#define UI_FIELD_V_Y     (4)
#define UI_FIELD_V_W     (120)   /* "-99.99" = 6 chars x 18 px at scale 3 + margin */
#define UI_FIELD_V_H     (21)

#define UI_FIELD_I_X     (170)
#define UI_FIELD_I_Y     (4)
#define UI_FIELD_I_W     (120)
#define UI_FIELD_I_H     (21)

#define UI_FIELD_M1_X    (36)
#define UI_FIELD_M1_Y    (34)
#define UI_FIELD_M2_X    (178)
#define UI_FIELD_M2_Y    (34)
#define UI_FIELD_M3_X    (36)
#define UI_FIELD_M3_Y    (51)
#define UI_FIELD_M4_X    (178)
#define UI_FIELD_M4_Y    (51)
#define UI_FIELD_M_W     (60)    /* "-9999" = 5 chars x 12 px at scale 2 (exact) */
#define UI_FIELD_M_H     (14)

#define UI_FIELD_P_X     (36)
#define UI_FIELD_P_Y     (68)
#define UI_FIELD_R_X     (132)
#define UI_FIELD_R_Y     (68)
#define UI_FIELD_Y_X     (228)
#define UI_FIELD_Y_Y     (68)
#define UI_FIELD_A_W     (48)    /* "-180" = 4 chars x 12 px (exact) */
#define UI_FIELD_A_H     (14)

#define UI_FIELD_D_X     (72)    /* "9999" = 4 chars x 12 px (exact) */
#define UI_FIELD_D_W     (48)
#define UI_FIELD_D_H     (14)
#define UI_FIELD_D_Y0    (92)    /* FRONT row; LEFT/RIGHT step +18 px */
#define UI_FIELD_D_STEP  (18)

#define UI_FIELD_UP_X    (32)    /* "4294967.29" = 10 chars x 6 px at scale 1 */
#define UI_FIELD_UP_Y    (148)
#define UI_FIELD_UP_W    (60)
#define UI_FIELD_UP_H    (7)

/**
 * @brief Clear one value field and redraw a fixed-point number (x100) in it,
 *        right-aligned so short numbers hug the field's right edge.
 */
static void TFT_UI_FieldFixed(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              int32_t val_scaled, uint8_t scale, uint16_t fg)
{
    char     buf[16];
    uint32_t mag;
    uint8_t  negative;
    uint16_t text_px;
    uint16_t xr;

    negative = (val_scaled < 0) ? 1U : 0U;
    mag = negative ? (uint32_t)(-val_scaled) : (uint32_t)val_scaled;
    sprintf(buf, "%s%lu.%02lu",
            negative ? "-" : "",
            (unsigned long)(mag / 100UL),
            (unsigned long)(mag % 100UL));

    TFT_SPI_FillRectangle(x, y, w, h, TFT_LCD_COLOR_BLACK);

    text_px = (uint16_t)(strlen(buf) * 6U * scale);
    xr = (text_px < w) ? (uint16_t)(x + w - text_px) : x;
    TFT_SPI_DrawString(xr, y, buf, fg, TFT_LCD_COLOR_BLACK, scale);
}

/**
 * @brief Clear one value field and redraw an integer in it, right-aligned.
 */
static void TFT_UI_FieldInt(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            int32_t val, uint8_t scale, uint16_t fg)
{
    char     buf[16];
    uint16_t text_px;
    uint16_t xr;

    sprintf(buf, "%ld", (long)val);

    TFT_SPI_FillRectangle(x, y, w, h, TFT_LCD_COLOR_BLACK);

    text_px = (uint16_t)(strlen(buf) * 6U * scale);
    xr = (text_px < w) ? (uint16_t)(x + w - text_px) : x;
    TFT_SPI_DrawString(xr, y, buf, fg, TFT_LCD_COLOR_BLACK, scale);
}

/**
 * @brief Draw the live dashboard (dirty-region refresh).
 */
void TFT_UI_ShowDashboard(void)
{
    SensorData_t d;
    int32_t      current_total_mA;
    int32_t      volt100;
    int32_t      cur100;
    int16_t      pitch_deg;
    int16_t      roll_deg;
    int16_t      yaw_deg;
    uint32_t     up_ms;
    uint8_t      i;

    SENSOR_GetHub(&d);

    /* Total current = sum of the four INA3221 motor channels. */
    current_total_mA = (int32_t)d.current_mA[0] + (int32_t)d.current_mA[1]
                     + (int32_t)d.current_mA[2] + (int32_t)d.current_mA[3];
    volt100 = (int32_t)d.bus_voltage_mV[0] / 10;
    cur100  = current_total_mA / 10;
    TFT_UI_QuatToEulerDeg(d.quat_w, d.quat_x, d.quat_y, d.quat_z,
                          &pitch_deg, &roll_deg, &yaw_deg);
    up_ms = TIMER_GetTick();

    /* First paint: clear the boot screen once and draw every static label.
       After that the labels are never touched again. */
    if (!s_dash_static_drawn)
    {
        TFT_SPI_FillScreen(TFT_LCD_COLOR_BLACK);

        TFT_SPI_DrawString(130, 4, "V", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 3);
        TFT_SPI_DrawString(294, 4, "A", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 3);

        TFT_SPI_DrawString(8, 34, "M1", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(150, 34, "M2", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(240, 34, "cm/s", TFT_LCD_COLOR_GRAY, TFT_LCD_COLOR_BLACK, 1);
        TFT_SPI_DrawString(8, 51, "M3", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(150, 51, "M4", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);

        TFT_SPI_DrawString(8, 68, "P", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(104, 68, "R", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(200, 68, "Y", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(284, 68, "deg", TFT_LCD_COLOR_GRAY, TFT_LCD_COLOR_BLACK, 1);

        TFT_SPI_DrawString(8, 92, "FRONT", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(124, 92, "mm", TFT_LCD_COLOR_GRAY, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(8, 110, "LEFT", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(124, 110, "mm", TFT_LCD_COLOR_GRAY, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(8, 128, "RIGHT", TFT_LCD_COLOR_WHITE, TFT_LCD_COLOR_BLACK, 2);
        TFT_SPI_DrawString(124, 128, "mm", TFT_LCD_COLOR_GRAY, TFT_LCD_COLOR_BLACK, 2);

        TFT_SPI_DrawString(8, 148, "UP", TFT_LCD_COLOR_GRAY, TFT_LCD_COLOR_BLACK, 1);
        TFT_SPI_DrawString(100, 148, "s", TFT_LCD_COLOR_GRAY, TFT_LCD_COLOR_BLACK, 1);

        s_dash_static_drawn = 1;
    }

    /* Value fields: clear+redraw only when the content changed. */
    if (s_lv_volt100 != volt100)
    {
        s_lv_volt100 = volt100;
        TFT_UI_FieldFixed(UI_FIELD_V_X, UI_FIELD_V_Y, UI_FIELD_V_W, UI_FIELD_V_H,
                          volt100, 3, TFT_LCD_COLOR_YELLOW);
    }
    if (s_lv_cur100 != cur100)
    {
        s_lv_cur100 = cur100;
        TFT_UI_FieldFixed(UI_FIELD_I_X, UI_FIELD_I_Y, UI_FIELD_I_W, UI_FIELD_I_H,
                          cur100, 3, TFT_LCD_COLOR_CYAN);
    }

    if (s_lv_speed[0] != d.speed_d100[0] / 100)
    {
        s_lv_speed[0] = d.speed_d100[0] / 100;
        TFT_UI_FieldInt(UI_FIELD_M1_X, UI_FIELD_M1_Y, UI_FIELD_M_W, UI_FIELD_M_H,
                        s_lv_speed[0], 2, TFT_LCD_COLOR_WHITE);
    }
    if (s_lv_speed[1] != d.speed_d100[1] / 100)
    {
        s_lv_speed[1] = d.speed_d100[1] / 100;
        TFT_UI_FieldInt(UI_FIELD_M2_X, UI_FIELD_M2_Y, UI_FIELD_M_W, UI_FIELD_M_H,
                        s_lv_speed[1], 2, TFT_LCD_COLOR_WHITE);
    }
    if (s_lv_speed[2] != d.speed_d100[2] / 100)
    {
        s_lv_speed[2] = d.speed_d100[2] / 100;
        TFT_UI_FieldInt(UI_FIELD_M3_X, UI_FIELD_M3_Y, UI_FIELD_M_W, UI_FIELD_M_H,
                        s_lv_speed[2], 2, TFT_LCD_COLOR_WHITE);
    }
    if (s_lv_speed[3] != d.speed_d100[3] / 100)
    {
        s_lv_speed[3] = d.speed_d100[3] / 100;
        TFT_UI_FieldInt(UI_FIELD_M4_X, UI_FIELD_M4_Y, UI_FIELD_M_W, UI_FIELD_M_H,
                        s_lv_speed[3], 2, TFT_LCD_COLOR_WHITE);
    }

    if (s_lv_pitch != pitch_deg)
    {
        s_lv_pitch = pitch_deg;
        TFT_UI_FieldInt(UI_FIELD_P_X, UI_FIELD_P_Y, UI_FIELD_A_W, UI_FIELD_A_H,
                        pitch_deg, 2, TFT_LCD_COLOR_WHITE);
    }
    if (s_lv_roll != roll_deg)
    {
        s_lv_roll = roll_deg;
        TFT_UI_FieldInt(UI_FIELD_R_X, UI_FIELD_R_Y, UI_FIELD_A_W, UI_FIELD_A_H,
                        roll_deg, 2, TFT_LCD_COLOR_WHITE);
    }
    if (s_lv_yaw != yaw_deg)
    {
        s_lv_yaw = yaw_deg;
        TFT_UI_FieldInt(UI_FIELD_Y_X, UI_FIELD_Y_Y, UI_FIELD_A_W, UI_FIELD_A_H,
                        yaw_deg, 2, TFT_LCD_COLOR_WHITE);
    }

    for (i = 0; i < 3; i++)
    {
        uint16_t dist = (i == 0) ? d.dist_front_mm :
                        (i == 1) ? d.dist_left_mm :
                                   d.dist_right_mm;
        if (s_lv_dist[i] != dist)
        {
            s_lv_dist[i] = dist;
            TFT_UI_FieldInt(UI_FIELD_D_X, (uint16_t)(UI_FIELD_D_Y0 + UI_FIELD_D_STEP * i),
                            UI_FIELD_D_W, UI_FIELD_D_H, dist, 2, TFT_LCD_COLOR_WHITE);
        }
    }

    if (s_lv_up_s100 != (int32_t)(up_ms / 10))
    {
        s_lv_up_s100 = (int32_t)(up_ms / 10);
        TFT_UI_FieldFixed(UI_FIELD_UP_X, UI_FIELD_UP_Y, UI_FIELD_UP_W, UI_FIELD_UP_H,
                          s_lv_up_s100, 1, TFT_LCD_COLOR_GREEN);
    }
}
