/**
 * @file  tft_spi.c
 * @brief ST7789-class 172x320 SPI display driver for the PEO rover data display.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Ported from the verified cube_solver_stm32f429vit6 TFT_SPI component and
 * reworked to the PEO data-display architecture (user-directed):
 *
 *   INIT PHASE - the screen is WRITTEN exactly once, 8-bit polled SPI3:
 *     pin/SPI setup, backlight/reset, full ST7789 startup sequence (kept
 *     byte-identical to the verified original), address window for the whole
 *     visible area, and a single RAMWR command.
 *
 *   DISPLAY PHASE - the screen is DRIVEN, CPU-free, forever:
 *     SPI3 flips to 16-bit frames and DMA1 Stream7 (SPI3_TX, Channel 0) pushes
 *     tft_spi_framebuf in CIRCULAR mode. NDTR (320*172 halfwords) matches the
 *     window pixel count exactly, and the panel's address counter wraps at the
 *     window end (Sitronix ST7789 datasheet 8.12) in lockstep with the DMA
 *     counter reload - no interrupt, no re-arming, D/C high, CS low.
 *
 *   CPU ROLE - pixels only: the Draw API (or direct framebuf writes) update
 *     memory; the next scan (<= 84 ms at 10.5 MHz) paints them. A region
 *     modified while the DMA scans it may show a one-scan mix (tearing) -
 *     invisible for dashboard text; chase the beam via DMA1_Stream7->NDTR
 *     if it ever matters.
 *
 * Display inversion 0x21 stays in the startup sequence exactly as in the
 * original; the panel's NET behavior is intuitive RGB565 (user-verified with
 * a four-quadrant calibration, see tft_spi.h), so TFT_LCD_COLOR_BLACK
 * (0x0000) shows black and text draws white-on-black by default.
 */
#include "include/tft_spi.h"
#include "include/tft_font5x7.h"
#include "COMPONENTS/DELAY/include/delay.h"

#include <stdio.h>

/* Whole-screen framebuffer, scanned by the circular DMA forever. ZI
   zero-init = black at boot. 110,080 bytes (~107.5 KB) of RAM. */
uint16_t tft_spi_framebuf[TFT_SPI_HEIGHT][TFT_SPI_WIDTH];

/* ============================================================================
   INIT-PHASE LOW-LEVEL BYTE TRANSPORT (8-bit polled, verified original behavior)
   ============================================================================ */

/**
 * @brief Send one byte with a software chip-select pulse around it.
 *
 * Kept identical to the verified driver: assert CS, wait idle, write DR,
 * drain the receive buffer (write-only 1-line mode), wait TXE then BSY,
 * release CS. The dummy DR read has no side effect on transmission.
 * Init phase only - once the DMA scan starts, CS never toggles again.
 * @param data Byte to send.
 */
static void TFT_SPI_TransferByte(uint8_t data)
{
    GPIO_ResetBits(PEO_PM_LCD_CS_PORT, PEO_PM_LCD_CS_PIN);            /* CS low */

    while (SPI_I2S_GetFlagStatus(PEO_PM_LCD_SPI, SPI_I2S_FLAG_BSY) != RESET);
    SPI_I2S_SendData(PEO_PM_LCD_SPI, data);
    (void)PEO_PM_LCD_SPI->DR;
    while (SPI_I2S_GetFlagStatus(PEO_PM_LCD_SPI, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(PEO_PM_LCD_SPI, SPI_I2S_FLAG_BSY) != RESET);

    GPIO_SetBits(PEO_PM_LCD_CS_PORT, PEO_PM_LCD_CS_PIN);              /* CS high */
}

/**
 * @brief Send an ST7789 command byte (D/C line low). Init phase only.
 * @param cmd Command opcode.
 */
static void TFT_SPI_WriteCommand(uint8_t cmd)
{
    GPIO_ResetBits(PEO_PM_LCD_DC_PORT, PEO_PM_LCD_DC_PIN);
    TFT_SPI_TransferByte(cmd);
}

/**
 * @brief Send one data byte (D/C line high). Init phase only.
 * @param data Data byte.
 */
static void TFT_SPI_WriteData8(uint8_t data)
{
    GPIO_SetBits(PEO_PM_LCD_DC_PORT, PEO_PM_LCD_DC_PIN);
    TFT_SPI_TransferByte(data);
}

/**
 * @brief Send one 16-bit data word, most significant byte first (D/C high).
 *        Init phase only - the DMA path pushes halfwords after the 16-bit flip.
 * @param data Data word.
 */
static void TFT_SPI_WriteData16(uint16_t data)
{
    GPIO_SetBits(PEO_PM_LCD_DC_PORT, PEO_PM_LCD_DC_PIN);
    TFT_SPI_TransferByte((uint8_t)(data >> 8));
    TFT_SPI_TransferByte((uint8_t)data);
}

/**
 * @brief Set the RAM address window; rows shift +34 to reach the visible area.
 *        Init phase only.
 */
static void TFT_SPI_SetFrameReference(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end)
{
    TFT_SPI_WriteCommand(0x2A);                 /* CASET: column address. */
    TFT_SPI_WriteData16(x_start);
    TFT_SPI_WriteData16(x_end);

    TFT_SPI_WriteCommand(0x2B);                 /* RASET: row address + GRAM offset. */
    TFT_SPI_WriteData16((uint16_t)(y_start + 34));
    TFT_SPI_WriteData16((uint16_t)(y_end + 34));
}

/* ============================================================================
   DISPLAY-PHASE DMA ENGINE
   ============================================================================ */

/**
 * @brief Poll the whole framebuffer (zero-init black) into the panel RAM.
 *
 * Init-phase only, 16-bit SPI3 frames, called with D/C high and CS low right
 * after the RAMWR command. Deterministically paints the GRAM black before
 * the backlight ever turns on - no scan-timing estimate involved. The panel
 * address counter wraps to the window start afterwards (ST7789 datasheet
 * 8.12), so the DMA scan continues seamlessly from the first pixel.
 */
static void TFT_SPI_ClearToBlack(void)
{
    uint32_t count = (uint32_t)TFT_SPI_WIDTH * TFT_SPI_HEIGHT;

    while (count--)
    {
        SPI_I2S_SendData(PEO_PM_LCD_SPI, 0x0000);
        while (SPI_I2S_GetFlagStatus(PEO_PM_LCD_SPI, SPI_I2S_FLAG_TXE) == RESET);
    }
    while (SPI_I2S_GetFlagStatus(PEO_PM_LCD_SPI, SPI_I2S_FLAG_BSY) != RESET);
}

/**
 * @brief Arm the circular DMA that scans the framebuffer into the panel forever.
 *
 * DMA1 Stream7 / Channel 0 is SPI3_TX (RM0090 Table 43). Halfword-to-halfword,
 * memory increment, peripheral fixed. NDTR = 320*172 = the window pixel count,
 * so the DMA reload and the panel's window wrap (ST7789 datasheet 8.12) stay
 * in lockstep. The DMA1 clock is enabled centrally by the RCC component, and
 * no interrupt is used, so NVIC needs nothing.
 */
static void TFT_SPI_DMA_Start(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    DMA_InitStruct.DMA_Channel            = DMA_Channel_0;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize         = (uint32_t)TFT_SPI_WIDTH * TFT_SPI_HEIGHT;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStruct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)&tft_spi_framebuf[0][0];
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&PEO_PM_LCD_SPI->DR;
    DMA_Init(DMA1_Stream7, &DMA_InitStruct);

    SPI_DMACmd(PEO_PM_LCD_SPI, SPI_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Stream7, ENABLE);
}

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief Configure the LCD GPIO pins and SPI3 (8-bit) from PEO_PINMAP.H section 10b.
 */
void TFT_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef  SPI_InitStruct;

    /* CS / RST / PWR: plain push-pull outputs on GPIOD, idle high (CS
       deselected, RST released, backlight off until Config turns it on). */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = PEO_PM_LCD_CS_PIN | PEO_PM_LCD_RST_PIN | PEO_PM_LCD_PWR_PIN;
    GPIO_Init(PEO_PM_LCD_CS_PORT, &GPIO_InitStruct);

    /* D/C on PB4: GPIO output (releases the NJTRST JTAG role). */
    GPIO_InitStruct.GPIO_Pin = PEO_PM_LCD_DC_PIN;
    GPIO_Init(PEO_PM_LCD_DC_PORT, &GPIO_InitStruct);

    /* SCL/SDA on PB3/PB5: alternate function AF6 = SPI3 (releases JTDO on PB3). */
    GPIO_PinAFConfig(PEO_PM_LCD_SCL_PORT, PEO_PM_LCD_SCL_SRC, PEO_PM_LCD_AF);
    GPIO_PinAFConfig(PEO_PM_LCD_SDA_PORT, PEO_PM_LCD_SDA_SRC, PEO_PM_LCD_AF);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin  = PEO_PM_LCD_SCL_PIN | PEO_PM_LCD_SDA_PIN;
    GPIO_Init(PEO_PM_LCD_SCL_PORT, &GPIO_InitStruct);

    /* SPI3 master, TX-only, 8-bit (init phase), MSB first, mode 3, /2
       prescaler: APB1 42 MHz / 2 = 21 MHz (full-screen scan 42 ms). The
       display phase re-flips the frame size to 16-bit. */
    SPI_InitStruct.SPI_Direction         = SPI_Direction_1Line_Tx;
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial     = 7;
    SPI_Init(PEO_PM_LCD_SPI, &SPI_InitStruct);

    /* Idle state: CS high (deselected), D/C high, RST high, backlight off. */
    GPIO_SetBits(PEO_PM_LCD_CS_PORT,  PEO_PM_LCD_CS_PIN);
    GPIO_SetBits(PEO_PM_LCD_DC_PORT,  PEO_PM_LCD_DC_PIN);
    GPIO_SetBits(PEO_PM_LCD_RST_PORT, PEO_PM_LCD_RST_PIN);
    GPIO_SetBits(PEO_PM_LCD_PWR_PORT, PEO_PM_LCD_PWR_PIN);
}

/**
 * @brief Power on the panel, run the full ST7789 startup sequence and hand the
 *        screen over to the circular DMA scan (the last software write here).
 */
void TFT_SPI_Config(void)
{
    SPI_Cmd(PEO_PM_LCD_SPI, ENABLE);

    TFT_SPI_BackLight(0);                       /* Keep the panel dark during reset. */
    TFT_SPI_HardReset();

    /* Backlight stays OFF through the whole command sequence and the first
       DMA scan: ST7789 does not clear its GRAM on reset, so turning the
       backlight on before the framebuffer (zero-init black) has been scanned
       in once would show random power-up stripes. */

    TFT_SPI_WriteCommand(0x11);                 /* Sleep out. */
    delay_ms(5);

    TFT_SPI_WriteCommand(0x36);                 /* MADCTL: display orientation. */
    TFT_SPI_WriteData8(0xA0);

    TFT_SPI_WriteCommand(0x3A);                 /* COLMOD: RGB565, 16 bits/pixel. */
    TFT_SPI_WriteData8(0x55);

    TFT_SPI_WriteCommand(0xB2);                 /* Porch control. */
    TFT_SPI_WriteData8(0x0C);
    TFT_SPI_WriteData8(0x0C);
    TFT_SPI_WriteData8(0x00);
    TFT_SPI_WriteData8(0x33);
    TFT_SPI_WriteData8(0x33);

    TFT_SPI_WriteCommand(0xB7);                 /* Gate control. */
    TFT_SPI_WriteData8(0x35);

    TFT_SPI_WriteCommand(0xBB);                 /* VCOM setting. */
    TFT_SPI_WriteData8(0x19);

    TFT_SPI_WriteCommand(0xC0);                 /* LCM control. */
    TFT_SPI_WriteData8(0x2C);

    TFT_SPI_WriteCommand(0xC2);                 /* VDV and VRH command enable. */
    TFT_SPI_WriteData8(0x01);

    TFT_SPI_WriteCommand(0xC3);                 /* VRH set. */
    TFT_SPI_WriteData8(0x12);

    TFT_SPI_WriteCommand(0xC4);                 /* VDV set. */
    TFT_SPI_WriteData8(0x20);

    TFT_SPI_WriteCommand(0xC6);                 /* Frame rate control (normal mode). */
    TFT_SPI_WriteData8(0x0F);

    TFT_SPI_WriteCommand(0xD0);                 /* Power control 1. */
    TFT_SPI_WriteData8(0xA4);
    TFT_SPI_WriteData8(0xA1);

    TFT_SPI_WriteCommand(0xE0);                 /* Positive voltage gamma. */
    TFT_SPI_WriteData8(0xF0);
    TFT_SPI_WriteData8(0x00);
    TFT_SPI_WriteData8(0x04);
    TFT_SPI_WriteData8(0x04);
    TFT_SPI_WriteData8(0x04);
    TFT_SPI_WriteData8(0x05);
    TFT_SPI_WriteData8(0x29);
    TFT_SPI_WriteData8(0x33);
    TFT_SPI_WriteData8(0x3E);
    TFT_SPI_WriteData8(0x38);
    TFT_SPI_WriteData8(0x12);
    TFT_SPI_WriteData8(0x12);
    TFT_SPI_WriteData8(0x28);
    TFT_SPI_WriteData8(0x30);

    TFT_SPI_WriteCommand(0xE1);                 /* Negative voltage gamma. */
    TFT_SPI_WriteData8(0xF0);
    TFT_SPI_WriteData8(0x07);
    TFT_SPI_WriteData8(0x0A);
    TFT_SPI_WriteData8(0x0D);
    TFT_SPI_WriteData8(0x0B);
    TFT_SPI_WriteData8(0x07);
    TFT_SPI_WriteData8(0x28);
    TFT_SPI_WriteData8(0x33);
    TFT_SPI_WriteData8(0x3E);
    TFT_SPI_WriteData8(0x36);
    TFT_SPI_WriteData8(0x14);
    TFT_SPI_WriteData8(0x14);
    TFT_SPI_WriteData8(0x29);
    TFT_SPI_WriteData8(0x32);

    TFT_SPI_WriteCommand(0x13);                 /* Normal display mode on. */
    TFT_SPI_WriteCommand(0x21);                 /* Inversion on (original sequence; net panel behavior is intuitive RGB565). */
    TFT_SPI_WriteCommand(0x29);                 /* Display on. */

    /* Address the whole visible area and open one RAMWR. This is the last
       time the screen is written; everything after is the DMA scan. */
    TFT_SPI_SetFrameReference(0, (uint16_t)(TFT_SPI_WIDTH - 1), 0, (uint16_t)(TFT_SPI_HEIGHT - 1));
    TFT_SPI_WriteCommand(0x2C);                 /* RAMWR: memory write. */

    /* Wait for the RAMWR byte to leave the shift register, then flip SPI3 to
       16-bit frames so the DMA can push RGB565 halfwords. */
    while (SPI_I2S_GetFlagStatus(PEO_PM_LCD_SPI, SPI_I2S_FLAG_BSY) != RESET);
    SPI_Cmd(PEO_PM_LCD_SPI, DISABLE);
    SPI_DataSizeConfig(PEO_PM_LCD_SPI, SPI_DataSize_16b);
    SPI_Cmd(PEO_PM_LCD_SPI, ENABLE);

    /* Hand over: D/C = data and CS = selected from now on. */
    GPIO_SetBits(PEO_PM_LCD_DC_PORT, PEO_PM_LCD_DC_PIN);
    GPIO_ResetBits(PEO_PM_LCD_CS_PORT, PEO_PM_LCD_CS_PIN);

    /* Deterministic power-up clear: the CPU polls black into the panel RAM
       so the GRAM is black BEFORE the backlight ever turns on. */
    TFT_SPI_ClearToBlack();

    /* Then the circular DMA takes over; the panel address counter wraps to
       the window start (ST7789 8.12), so the scan continues seamlessly and
       the backlight lights a clean black screen. */
    TFT_SPI_DMA_Start();
    TFT_SPI_BackLight(1);
}

/**
 * @brief Switch the backlight (PWR pin drives a PMOS gate, active-low).
 */
void TFT_SPI_BackLight(uint8_t on)
{
    if (on)
    {
        GPIO_ResetBits(PEO_PM_LCD_PWR_PORT, PEO_PM_LCD_PWR_PIN);   /* Low = ON. */
    }
    else
    {
        GPIO_SetBits(PEO_PM_LCD_PWR_PORT, PEO_PM_LCD_PWR_PIN);     /* High = OFF. */
    }
}

/**
 * @brief Pulse the RST line low for 120 ms, then release it for 120 ms.
 */
void TFT_SPI_HardReset(void)
{
    GPIO_ResetBits(PEO_PM_LCD_RST_PORT, PEO_PM_LCD_RST_PIN);
    delay_ms(120);

    GPIO_SetBits(PEO_PM_LCD_RST_PORT, PEO_PM_LCD_RST_PIN);
    delay_ms(120);
}

/* ============================================================================
   FRAMEBUFFER DRAW API (memory only; the DMA scan paints within ~84 ms)
   ============================================================================ */

/**
 * @brief Fill one rectangle of the framebuffer with one color (clipped).
 */
void TFT_SPI_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint16_t row;
    uint16_t col;

    if (x >= TFT_SPI_WIDTH || y >= TFT_SPI_HEIGHT)
    {
        return;                                 /* Entirely off-screen. */
    }
    if ((uint32_t)x + w > TFT_SPI_WIDTH)
    {
        w = (uint16_t)(TFT_SPI_WIDTH - x);
    }
    if ((uint32_t)y + h > TFT_SPI_HEIGHT)
    {
        h = (uint16_t)(TFT_SPI_HEIGHT - y);
    }

    for (row = 0U; row < h; row++)
    {
        for (col = 0U; col < w; col++)
        {
            tft_spi_framebuf[y + row][x + col] = color;
        }
    }
}

/**
 * @brief Fill the whole framebuffer with one color.
 */
void TFT_SPI_FillScreen(uint16_t color)
{
    TFT_SPI_FillRectangle(0, 0, TFT_SPI_WIDTH, TFT_SPI_HEIGHT, color);
}

/**
 * @brief Write one pixel into the framebuffer.
 */
void TFT_SPI_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x < TFT_SPI_WIDTH && y < TFT_SPI_HEIGHT)
    {
        tft_spi_framebuf[y][x] = color;
    }
}

/**
 * @brief Draw one 5x7 ASCII character into the framebuffer, optionally scaled.
 */
void TFT_SPI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint8_t col;
    uint8_t row;

    if (scale == 0U)
    {
        scale = 1U;
    }
    if ((uint8_t)c < TFT_FONT5X7_FIRST || (uint8_t)c > TFT_FONT5X7_LAST)
    {
        c = '?';
    }

    for (col = 0U; col < 5U; col++)
    {
        uint8_t bits = TFT_FONT5X7[(uint8_t)c - TFT_FONT5X7_FIRST][col];
        for (row = 0U; row < 7U; row++)
        {
            uint16_t color = (bits & (1U << row)) ? fg : bg;
            TFT_SPI_FillRectangle((uint16_t)(x + col * scale),
                                  (uint16_t)(y + row * scale),
                                  scale, scale, color);
        }
    }
}

/**
 * @brief Draw a NUL-terminated string into the framebuffer, advancing 6
 *        columns per character.
 */
void TFT_SPI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*str != '\0')
    {
        TFT_SPI_DrawChar(x, y, *str, fg, bg, scale);
        x = (uint16_t)(x + 6U * scale);
        str++;
    }
}

/**
 * @brief Draw a signed decimal number into the framebuffer as text.
 */
void TFT_SPI_DrawNumber(uint16_t x, uint16_t y, int32_t num, uint16_t fg, uint16_t bg, uint8_t scale)
{
    char buf[12];

    sprintf(buf, "%ld", (long)num);
    TFT_SPI_DrawString(x, y, buf, fg, bg, scale);
}

/**
 * @brief Pack 8-8-8 RGB channels into a 16-bit RGB565 RAM value.
 */
uint16_t TFT_SPI_RGB565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t r5 = ((uint16_t)r >> 3) & 0x1FU;   /* 0..31 */
    uint16_t g6 = ((uint16_t)g >> 2) & 0x3FU;   /* 0..63 */
    uint16_t b5 = ((uint16_t)b >> 3) & 0x1FU;   /* 0..31 */

    return (uint16_t)((r5 << 11) | (g6 << 5) | b5);
}
