/**
 * @file  tft_spi.h
 * @brief ST7789-class 172x320 SPI display driver for the PEO rover data display.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Ported from the verified TFT_SPI component of the cube_solver_stm32f429vit6
 * project, which ran on this same dev-board LCD FPC. The panel is a
 * 172x320 IPS display with an ST7789-class controller, driven write-only:
 *
 *     SCL = PB3 (SPI3_SCK,  AF6)          CS  = PD7 (GPIO)
 *     SDA = PB5 (SPI3_MOSI, AF6)          RST = PD4 (GPIO, active-low)
 *     DC  = PB4 (GPIO output)             PWR = PD3 (GPIO, backlight PMOS Q1,
 *                                                   active-low: low = ON)
 *
 * DRIVER ARCHITECTURE (user-directed, 2026-11-20):
 *   - Init phase: 8-bit polled SPI3 writes only (panel reset + ST7789 command
 *     sequence + address window + one RAMWR). The screen is "written" exactly
 *     once; from then on it is only driven.
 *   - Display phase: SPI3 flips to 16-bit frames and DMA1 Stream7 (SPI3_TX)
 *     pushes tft_spi_framebuf in CIRCULAR mode forever. The panel's address
 *     counter wraps at the window end (Sitronix ST7789 datasheet 8.12) in
 *     lockstep with the DMA counter reload, so no interrupt or CPU action is
 *     ever needed again: D/C stays high, CS stays low.
 *   - CPU role: only write pixels into tft_spi_framebuf (via the Draw API or
 *     directly); the next scan (<= 42 ms later at 21 MHz) paints them.
 *     Tearing: a region modified WHILE the DMA scans it shows a mix for one
 *     scan. For dashboard text this is invisible; if it ever matters, chase
 *     the beam by reading DMA1_Stream7->NDTR before writing a row.
 *
 * COLOR SEMANTICS (user-verified on this panel, 2026-11-20, four-quadrant
 * calibration): the RAM values behave as INTUITIVE RGB565 - 0x0000 shows
 * black, 0xFFFF white, 0xF800 red, 0x07E0 green. The original project's
 * inversion-based constants (0xFFFF = "black") do NOT apply to this panel.
 * The verified startup sequence keeps 0x21 (INVON) exactly as the original;
 * the net panel behavior is intuitive RGB565, so the color table below is
 * the plain RGB565 one. Do not "fix" it back to inverted values.
 *
 * ORIENTATION (user-verified): with the board held so the RGB LED sits at
 * the observer's top-right, display coordinate (0,0) is the observer's
 * top-left corner; x grows rightward, y downward. If the screen is ever
 * remounted in another orientation, adjust MADCTL (0x36) instead of the
 * drawing coordinates.
 */
#ifndef __TFT_SPI_H__
#define __TFT_SPI_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* Panel geometry: 320 wide x 172 tall visible area (ST7789 240x320 GRAM,
   visible window starts at GRAM row 34; see TFT_SPI_SetFrameReference). */
#define TFT_SPI_WIDTH           (320)
#define TFT_SPI_HEIGHT          (172)

/* Whole-screen framebuffer the circular DMA scans forever. RGB565, row-major,
   index [y][x]; [0][0] is the observer's top-left (see orientation note).
   ZI zero-init means the panel boots to black before any drawing happens.
   CPU writes land on screen within one scan period (~42 ms at 21 MHz). */
extern uint16_t tft_spi_framebuf[TFT_SPI_HEIGHT][TFT_SPI_WIDTH];

/* Verified panel colors (intuitive RGB565 - see file header). */
#define TFT_LCD_COLOR_BLACK     (0x0000)  /* RAM 0x0000 displays black. */
#define TFT_LCD_COLOR_WHITE     (0xFFFF)  /* RAM 0xFFFF displays white. */
#define TFT_LCD_COLOR_RED       (0xF800)
#define TFT_LCD_COLOR_GREEN     (0x07E0)
#define TFT_LCD_COLOR_BLUE      (0x001F)
#define TFT_LCD_COLOR_YELLOW    (0xFFE0)
#define TFT_LCD_COLOR_CYAN      (0x07FF)
#define TFT_LCD_COLOR_MAGENTA   (0xF81F)
#define TFT_LCD_COLOR_GRAY      (0x8410)  /* Half-brightness gray for annotations. */

/**
 * @brief Configure the LCD GPIO pins and SPI3 peripheral from PEO_PINMAP.H.
 *
 * Releases JTAG on PB3/PB4 (SWD on PA13/PA14 stays available) and leaves the
 * panel deselected. Clock and NVIC are owned by the central RCC/NVIC components.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void TFT_SPI_Init(void);

/**
 * @brief Power on the panel: run the full ST7789 startup sequence, start the
 *        circular DMA scan and switch the backlight on once the first black
 *        scan has covered the power-up GRAM.
 *
 * main() calls TFT_SPI_Init() early (killing the floating backlight gate as
 * soon as possible) and TFT_SPI_Config() after the delay hardware is up.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void TFT_SPI_Config(void);

/**
 * @brief Pulse the RST line low for 120 ms, then release it for 120 ms.
 *
 * The panel is reset as part of TFT_SPI_Config(); call this directly only for
 * an out-of-band reset.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void TFT_SPI_HardReset(void);

/**
 * @brief Switch the panel backlight.
 *
 * @param on 1 = backlight on, 0 = off (PWR pin is a PMOS gate, active-low).
 * @retval None This function does not return a value.
 */
void TFT_SPI_BackLight(uint8_t on);

/**
 * @brief Set the RAM write window (column/row address range).
 *
 * Rows are shifted +34 inside the driver to reach the panel's visible area;
 * pass VISIBLE coordinates (0..TFT_SPI_WIDTH-1, 0..TFT_SPI_HEIGHT-1).
 * @param x_start First visible column.
 * @param x_end   Last visible column.
 * @param y_start First visible row.
 * @param y_end   Last visible row.
 * @retval None This function does not return a value.
 */
void TFT_SPI_SetFrameReference(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end);

/**
 * @brief Fill the whole visible screen with one color.
 *
 * @param color 16-bit RAM color value (see TFT_LCD_COLOR_* constants).
 * @retval None This function does not return a value.
 */
void TFT_SPI_FillScreen(uint16_t color);

/**
 * @brief Fill one rectangle with one color.
 *
 * @param x     Left edge (visible column).
 * @param y     Top edge (visible row).
 * @param w     Width in pixels.
 * @param h     Height in pixels.
 * @param color 16-bit RAM color value.
 * @retval None This function does not return a value.
 */
void TFT_SPI_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief Draw one pixel.
 *
 * @param x     Visible column.
 * @param y     Visible row.
 * @param color 16-bit RAM color value.
 * @retval None This function does not return a value.
 */
void TFT_SPI_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw one 5x7 ASCII character, optionally scaled.
 *
 * The character cell is 6 columns wide (5 glyph + 1 spacing) and 7 rows tall,
 * both multiplied by scale. Background pixels are painted with bg.
 * @param x     Left edge of the cell (visible column).
 * @param y     Top edge of the cell (visible row).
 * @param c     ASCII character (0x20..0x7E; others render as '?').
 * @param fg    Foreground (glyph) color.
 * @param bg    Background color.
 * @param scale Integer magnification, 1..3 typical.
 * @retval None This function does not return a value.
 */
void TFT_SPI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);

/**
 * @brief Draw a NUL-terminated ASCII string, optionally scaled.
 *
 * @param x     Left edge of the first cell (visible column).
 * @param y     Top edge (visible row).
 * @param str   NUL-terminated string (no line wrapping).
 * @param fg    Foreground (glyph) color.
 * @param bg    Background color.
 * @param scale Integer magnification, 1..3 typical.
 * @retval None This function does not return a value.
 */
void TFT_SPI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale);

/**
 * @brief Draw a signed decimal number as text.
 *
 * @param x     Left edge of the first cell (visible column).
 * @param y     Top edge (visible row).
 * @param num   Signed 32-bit value to print.
 * @param fg    Foreground (glyph) color.
 * @param bg    Background color.
 * @param scale Integer magnification, 1..3 typical.
 * @retval None This function does not return a value.
 */
void TFT_SPI_DrawNumber(uint16_t x, uint16_t y, int32_t num, uint16_t fg, uint16_t bg, uint8_t scale);

/**
 * @brief Pack 8-8-8 RGB channels into a 16-bit RGB565 RAM value.
 *
 * @note The panel inverts the display (see file header), so the DISPLAYED hue
 *       is the complement of the packed value. Use the TFT_LCD_COLOR_*
 *       constants for the verified common colors.
 * @param r Red channel 0..255.
 * @param g Green channel 0..255.
 * @param b Blue channel 0..255.
 * @retval uint16_t Packed RGB565 value.
 */
uint16_t TFT_SPI_RGB565(uint8_t r, uint8_t g, uint8_t b);

#endif /* __TFT_SPI_H__ */
