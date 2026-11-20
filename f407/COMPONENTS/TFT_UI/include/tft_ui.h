/**
 * @file  tft_ui.h
 * @brief On-screen views for the TFT_SPI display (boot screen, dashboard).
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * UI layer above the TFT_SPI driver: composes text layouts out of the
 * driver's Draw API (which only writes the framebuffer; the circular DMA
 * scan paints them). The boot screen mirrors the serial boot banner; the
 * dashboard will extend this component.
 */
#ifndef __TFT_UI_H__
#define __TFT_UI_H__

/**
 * @brief Draw the boot screen: title, firmware version, build date, module
 *        build-switch states and a green BOOT OK line.
 *
 * Call after all component inits (next to print_boot_banner) so the module
 * states shown are the final ones. The framebuffer is zero-init black, so no
 * clear is needed before drawing.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void TFT_UI_ShowBoot(void);

/**
 * @brief Draw the live dashboard: bus voltage / total current on top, four
 *        wheel speeds and attitude (pitch/roll/yaw from the MPU quaternion)
 *        in the middle, and the three VL53L1X distances at the bottom.
 *
 * Pulls one consistent snapshot via SENSOR_GetHub(), clears the framebuffer
 * and redraws everything. Call periodically from the main loop (the firmware
 * refreshes every 500 ms; the boot screen stays visible ~2 s first).
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void TFT_UI_ShowDashboard(void);

#endif /* __TFT_UI_H__ */
