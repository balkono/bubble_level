//MIT License
//
//Copyright (c) 2024 Moritz Emersberger
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

/*$T src/ssd1351.h GC 1.150 2023-03-18 10:52:22 */

//MIT License
//Copyright (c) 2022 Jaime Centeno
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
// to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#ifndef SSD1351_H
#define SSD1351_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>

#include "fonts.h"
#include "sprites.h"
#include "color_palette.h"

/*------- CONFIGURE THIS TO YOUR OWN HARDWARE AND HARDWARE ABSTRACTION LAYER -------*/

// Provide your own HAL SPI Communication definitions in myHAL.h/myHAL.c as
// SPI_TXByte(data) and SPI_TXBuffer(buffer, len)
// and GPIO_SetPin(PORT, PIN) GPIO_ResetPin(PORT, PIN)
#include "myHAL.h"

// Define the ports and pins for your hardware
// following your HAL ports/pins definitions
// In this example:
// PC14 = RESET      PB1 = D/C#      PB0 = CS

#define RESET_PORT GPIOC
#define RESET_PIN GPIO_PIN_14
#define DC_PORT GPIOB
#define DC_PIN  GPIO_PIN_1
#define CS_PORT GPIOB
#define CS_PIN GPIO_PIN_0

#define OLED_96x96

// Definitions for SPI functions
#define SSD1351_SendBuffer(buffer, len) SPI_TXBuffer(buffer, len)
#define SSD1351_SendByte(data)			SPI_TXByte(data)

// Definitions for GPIO pin functions
#define SSD1351_SetPin(PORT, PIN)	GPIO_SetPin(PORT, PIN)
#define SSD1351_ClearPin(PORT, PIN) GPIO_ResetPin(PORT, PIN)

// Definition for delay function
#define SSD1351_DelayMs(x)	HAL_Delay(x)

/*------------------- END OF HARDWARE CONFIGURATION ------------------------------- */

// Static definition for testing purposes on Ceedling
#ifdef TEST
#define STATIC
#else
#define STATIC	static
#endif // TEST
#ifdef OLED_128x128
#define COLUMNS			128
#define ROWS			128
#endif // OLED_128x128
#ifdef OLED_96x96
#define COLUMNS         96
#define ROWS            96
#define BUF_SIZE        ROWS * COLUMNS
#endif // OLED_96x96


typedef union DisplayRAM
{
	uint16_t     twobcolour[BUF_SIZE];
} DRAM;

// SSD1351 Commands
#define SSD1351_CMD_SETCOLUMN		0x15
#define SSD1351_CMD_SETROW			0x75
#define SSD1351_CMD_WRITERAM		0x5C
#define SSD1351_CMD_READRAM			0x5D
#define SSD1351_CMD_SETREMAP		0xA0
#define SSD1351_CMD_STARTLINE		0xA1
#define SSD1351_CMD_DISPLAYOFFSET	0xA2
#define SSD1351_CMD_DISPLAYALLOFF	0xA4
#define SSD1351_CMD_DISPLAYALLON	0xA5
#define SSD1351_CMD_NORMALDISPLAY	0xA6
#define SSD1351_CMD_INVERTDISPLAY	0xA7
#define SSD1351_CMD_FUNCTIONSELECT	0xAB
#define SSD1351_CMD_DISPLAYOFF		0xAE
#define SSD1351_CMD_DISPLAYON		0xAF
#define SSD1351_CMD_PRECHARGE		0xB1
#define SSD1351_CMD_DISPLAYENHANCE	0xB2
#define SSD1351_CMD_CLOCKDIV		0xB3
#define SSD1351_CMD_SETVSL			0xB4
#define SSD1351_CMD_SETGPIO			0xB5
#define SSD1351_CMD_PRECHARGE2		0xB6
#define SSD1351_CMD_SETGRAY			0xB8
#define SSD1351_CMD_USELUT			0xB9
#define SSD1351_CMD_PRECHARGELEVEL	0xBB
#define SSD1351_CMD_VCOMH			0xBE
#define SSD1351_CMD_CONTRASTABC		0xC1
#define SSD1351_CMD_CONTRASTMASTER	0xC7
#define SSD1351_CMD_MUXRATIO		0xCA
#define SSD1351_CMD_COMMANDLOCK		0xFD
#define SSD1351_CMD_HORIZSCROLL		0x96
#define SSD1351_CMD_STOPSCROLL		0x9E
#define SSD1351_CMD_STARTSCROLL		0x9F

// Some colour definitions
#define COLOUR_BLUE		0x1F00
#define COLOUR_RED		0x00F8
#define COLOUR_GREEN	0xE007
#define COLOUR_YELLOW	0xE0FF
#define COLOUR_PURPLE	0x1FF8
#define COLOUR_AQUA		0xFF07
#define COLOUR_BLACK	0x0000
#define COLOUR_WHITE	0xFFFF
#define COLOUR_GREY     0x7BEF

#define SSD_PRINTF(...) SSD1351_printf(COLOUR_WHITE, small_font, __VA_ARGS__)

extern struct cursor	SSD1351_cursor;

uint16_t				SSD1351_get_rgb(uint8_t r, uint8_t g, uint8_t b);

void					SSD1351_init(void);

void                    SSD1351_all_on(void);

void                    SSD1351_normalmode(void);

void                    SSD1351_display_on(void);

void                    SSD1351_display_off(void);

void					SSD1351_fill65k(uint16_t colour);

void					SSD1351_write_pixel(int16_t x, int16_t y, uint16_t colour);

void					SSD1351_update(void);

void					SSD1351_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour);

void					SSD1351_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t colour);

void					SSD1351_draw_filled_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t colour);

void					SSD1351_draw_circle(int16_t xc, int16_t yc, uint16_t r, uint16_t colour);

void					SSD1351_draw_filled_circle(int16_t xc, int16_t yc, uint16_t r, uint16_t colour);

void					SSD1351_printf(uint16_t colour, const font_t *font, const char *format, ...);

void					SSD1351_set_cursor(uint8_t x, uint8_t y);

uint8_t                 SSD1351_get_cursor_x(void);

uint8_t                 SSD1351_get_cursor_y(void);

void					SSD1351_draw_sprite(int16_t x, int16_t y, sprite *sp);

#endif //SSD1351_H
