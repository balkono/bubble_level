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

/*$T src/ssd1351.c GC 1.150 2023-03-18 10:52:19 */


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

#include "ssd1351.h"

/* Buffer to hold the Display RAM Data */
STATIC DRAM displayRAM;
//STATIC DRAM displayRAM_old;

#define DRAM_2b     displayRAM.twobcolour

/* Screen cursor for printing */
struct cursor
{
	uint8_t x;
	uint8_t y;
} SSD1351_cursor;

/**
  * @brief  Writes command to the SSD1351 OLED Display
  * @param  cmd: command to send
  * @retval None
  */
STATIC void SSD1351_write_command(uint8_t cmd)
{
	SSD1351_ClearPin(DC_PORT, DC_PIN);
	SSD1351_ClearPin(CS_PORT, CS_PIN);
	SSD1351_SendByte(cmd);
	SSD1351_SetPin(CS_PORT, CS_PIN);
}

/**
  * @brief  Writes single byte data to the SSD1351 OLED Display
  * @param  data: data byte to send
  * @retval None
  */
STATIC void SSD1351_write_data(uint8_t data)
{
	SSD1351_SetPin(DC_PORT, DC_PIN);
	SSD1351_ClearPin(CS_PORT, CS_PIN);
	SSD1351_SendByte(data);
	SSD1351_SetPin(CS_PORT, CS_PIN);
}

/**
  * @brief  Writes a data buffer of bytes to SSD1351 display
  * @param  data: pointer to data buffer to send
  * @param  len: integer with length of buffer to send
  * @retval None
  */
STATIC void SSD1351_write_data_buffer(uint16_t* data, uint32_t len)
{
	SSD1351_SetPin(DC_PORT, DC_PIN);
	SSD1351_ClearPin(CS_PORT, CS_PIN);
	SSD1351_SendBuffer(data, len);
	SSD1351_SetPin(CS_PORT, CS_PIN);
}

/*
 * @brief Converts from RGB to a single 16bit value
 * @param r: red
 * @param g: green
 * @param b: blue
 * @retval 16bit value with the rgb colour for display
 */
uint16_t SSD1351_get_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t rgb_colour = 0;
	r= r>>3;
	g= g>>2;
	b= b>>3;
	rgb_colour = (r<<3) | ((g&0x38)>>3) | ((g&0x07)<<13)| (b<<8);
	return rgb_colour;
}

/**
  * @brief  Initializes the SSD1351 OLED Display
  * @retval None
  */
void SSD1351_init(void)
{
	SSD1351_write_command(SSD1351_CMD_COMMANDLOCK);
	SSD1351_write_data(0x12); //unlock

	SSD1351_write_command(SSD1351_CMD_DISPLAYOFF);

	SSD1351_write_command(SSD1351_CMD_MUXRATIO);
	SSD1351_write_data(96); //96 to enable 96 common lines
	SSD1351_DelayMs(600);

	SSD1351_write_command(SSD1351_CMD_SETREMAP); //0x2 for seg0 mapped to address 127
	SSD1351_write_data(0x22); //colour depth, 0x20 for 65k and COM split odd even

	SSD1351_write_command(SSD1351_CMD_SETCOLUMN); //range for 96 colums acc. to psp27801
	SSD1351_write_data(0x10); //16
	SSD1351_write_data(0x6F); //111

	SSD1351_write_command(SSD1351_CMD_SETROW);//range for 96 rows acc. to psp27801
	SSD1351_write_data(0x20); //32
	SSD1351_write_data(0x7F); //127

	SSD1351_write_command(SSD1351_CMD_STARTLINE);
	SSD1351_write_data(0x20); //vertical scroll (RAM) offset 32

	SSD1351_write_command(SSD1351_CMD_DISPLAYOFFSET);
	SSD1351_write_data(0x60); //default

	SSD1351_write_command(SSD1351_CMD_SETGPIO);
	SSD1351_write_data(0x00);//GPIOs disabled

	SSD1351_write_command(SSD1351_CMD_FUNCTIONSELECT);
	SSD1351_write_data(0x01);//internal VDD

	SSD1351_write_command(SSD1351_CMD_PRECHARGE);
	SSD1351_write_data(0x82); //reset value

	SSD1351_write_command(SSD1351_CMD_VCOMH);
	SSD1351_write_data(0x05); //reset value

	SSD1351_write_command(SSD1351_CMD_NORMALDISPLAY); //set display off

	SSD1351_write_command(SSD1351_CMD_CONTRASTABC); //default values
	SSD1351_write_data(0x8A);	// colour A: Red
	SSD1351_write_data(0x51);	// colour B: Green
	SSD1351_write_data(0x8A);	// colour C: Blue
	SSD1351_write_command(SSD1351_CMD_CONTRASTMASTER);
	SSD1351_write_data(0x0F); //no change

	SSD1351_write_command(SSD1351_CMD_SETVSL);
	SSD1351_write_data(0xA0);
	SSD1351_write_data(0xB5);
	SSD1351_write_data(0x55);

	SSD1351_write_command(SSD1351_CMD_PRECHARGE2);
	SSD1351_write_data(0x04);

	SSD1351_write_command(SSD1351_CMD_DISPLAYON);
}

void SSD1351_display_on()
{
    SSD1351_write_command(SSD1351_CMD_DISPLAYON);
}

void SSD1351_display_off()
{
    SSD1351_write_command(SSD1351_CMD_DISPLAYOFF);
}

void SSD1351_all_on()
{
    SSD1351_write_command(SSD1351_CMD_DISPLAYALLON);
}

void SSD1351_normalmode()
{
    SSD1351_write_command(SSD1351_CMD_NORMALDISPLAY);
}

void SSD1351_fill65k(uint16_t colour)
{
    for(int i = 0; i < BUF_SIZE; i++)
    {
        DRAM_2b[i] = colour;
    }
}

/**
  * @brief  Updates the screen RAM
  * @retval None
  */
void SSD1351_update(void)
{
	SSD1351_write_command(SSD1351_CMD_WRITERAM);
	SSD1351_write_data_buffer(DRAM_2b, BUF_SIZE);
	//SSD1351_write_command(SSD1351_CMD_STOPSCROLL);
}

/**
 * @brief Writes a pixel data to the screen RAM buffer
 * @param colour: Unsigned int16 containing colour code
 * @param x: Pixel's horizontal position
 * @param y: Pixel's vertical position
 * @retval None
 */
void SSD1351_write_pixel(int16_t x, int16_t y, uint16_t colour)
{
	if(x > 95 || y > 95 || x < 0 || y < 0)
	{
		return;
	}

	int a = x + (y * 96);
	DRAM_2b[a] = colour;
}

/*  LINE DRAWING FUNCTIONS */
STATIC void SSD1351_draw_line_low(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour)
{
	int16_t dx = x1 - x0;
	int16_t dy = y1 - y0;
	int16_t yi = 1;
	if(dy < 0)
	{
		yi = -1;
		dy = -dy;
	}

	int16_t D = 2 * dy - dx;
	int16_t y = y0;

	if(x0 < x1)
	{
		for(int16_t x = x0; x <= x1; x++)
		{
			SSD1351_write_pixel(x, y, colour);
			if(D > 0)
			{
				y = y + yi;
				D = D - 2 * dx;
			}

			D = D + 2 * dy;
		}
	}
	else
	{
		for(int16_t x = x0; x >= x1; x--)
		{
			SSD1351_write_pixel(x, y, colour);
			if(D > 0)
			{
				y = y + yi;
				D = D - 2 * dx;
			}

			D = D + 2 * dy;
		}
	}
}

/* */
STATIC void SSD1351_draw_line_high(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour)
{
	int16_t dx = x1 - x0;
	int16_t dy = y1 - y0;
	int16_t xi = 1;
	if(dx < 0)
	{
		xi = -1;
		dx = -dx;
	}

	int16_t D = 2 * dx - dy;
	int16_t x = x0;

	if(y0 < y1)
	{
		for(int16_t y = y0; y <= y1; y++)
		{
			SSD1351_write_pixel(x, y, colour);
			if(D > 0)
			{
				x = x + xi;
				D = D - 2 * dy;
			}

			D = D + 2 * dx;
		}
	}
	else
	{
		for(int16_t y = y0; y >= y1; y--)
		{
			SSD1351_write_pixel(x, y, colour);
			if(D > 0)
			{
				x = x + xi;
				D = D - 2 * dy;
			}

			D = D + 2 * dx;
		}
	}
}

/*
 * @brief Draws a line from specified parameters into display RAM
 * @param x0: starting x coordinate
 * @param y0: starting y coordinate
 * @param x1: ending x coordinate
 * @param y1: ending y coordinate
 * @colour: colour to use to draw the line
 * @retval None
 */
void SSD1351_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t colour)
{
	if(abs(y1 - y0) < abs(x1 - x0))
	{
		if(x0 > x1)
		{
			SSD1351_draw_line_low(x1, y1, x0, y0, colour);
		}
		else
		{
			SSD1351_draw_line_low(x0, y0, x1, y1, colour);
		}
	}
	else
	{
		if(y0 > y1)
		{
			SSD1351_draw_line_high(x1, y1, x0, y0, colour);
		}
		else
		{
			SSD1351_draw_line_high(x0, y0, x1, y1, colour);
		}
	}

	return;
}

/*
 * @brief Draws a rectangle with specified dimensions into display RAM
 * @param x0: starting x coordinate
 * @param y0: starting y coordinate
 * @param w: width of the rectangle
 * @oaram h: height of the rectangle
 * @colour: colour for the border
 * @retval None
 */
void SSD1351_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t colour)
{
	SSD1351_draw_line(x, y, x + (w-1), y, colour);
	SSD1351_draw_line(x + (w-1), y, x + (w-1), y + (h-1), colour);
	SSD1351_draw_line(x + (w-1), y + (h-1), x, y + (h-1), colour);
	SSD1351_draw_line(x, y + (h-1), x, y, colour);
}

/*
 * @brief Draws a filled rectangle with specified dimensions into display RAM
 * @param x0: starting x coordinate
 * @para y0: starting y coordinate
 * @param w: width of the rectangle
 * @oaram h: height of the rectangle
 * @oaram colour: colour for the rectangle
 * @retval None
 */
void SSD1351_draw_filled_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t colour)
{
	for(int i = x; i < x + w; i++)
	{
		for(int j = y; j < y + h; j++)
		{
			SSD1351_write_pixel(i, j, colour);
		}
	}
}

/*
 * @brief Draws a rotated filled rectangle with specified dimensions into display RAM
 * @param xc: center x coordinate
 * @para yc: center y coordinate
 * @param w: width of the rectangle
 * @param h: height of the rectangle
 * @param r: rotation in degrees
 * @oaram colour: colour for the rectangle
 * @retval None
 */

/*
void SSD1351_draw_rotated_rect(int16_t xc, int16_t yc, int16_t w, int16_t h, int16_t r, uint16_t colour){
  r = r % 90;
  int16_t hyp = sqrt((h * h)/4 + (w * w)/4);
  float wr = 45  + r;
  float rad = (wr/360)*M_PI;
  int16_t x0 = xc - (hyp * sin(rad));
  int16_t y0 = yc + (hyp * cos(rad));
}*/
STATIC void draw_circle(int16_t xc, int16_t yc, int16_t x, int16_t y, uint16_t colour)
{
	SSD1351_write_pixel(xc + x, yc + y, colour);
	SSD1351_write_pixel(xc - x, yc + y, colour);
	SSD1351_write_pixel(xc + x, yc - y, colour);
	SSD1351_write_pixel(xc - x, yc - y, colour);
	SSD1351_write_pixel(xc + y, yc + x, colour);
	SSD1351_write_pixel(xc - y, yc + x, colour);
	SSD1351_write_pixel(xc + y, yc - x, colour);
	SSD1351_write_pixel(xc - y, yc - x, colour);
}

/* */
STATIC void draw_filled_circle(int16_t xc, int16_t yc, int16_t x, int16_t y, uint16_t colour)
{
	SSD1351_draw_line(xc - x, yc + y, xc + x, yc + y, colour);
	SSD1351_draw_line(xc - x, yc - y, xc + x, yc - y, colour);
	SSD1351_draw_line(xc - y, yc + x, xc + y, yc + x, colour);
	SSD1351_draw_line(xc - y, yc - x, xc + y, yc - x, colour);
}

/*
 * @brief Draws a cicle with specified origin and radius into display RAM
 * @param xc: integer for the x origin coordinate
 * @param yc: integer for the y origin coordinate
 * @param colour: colour for the border
 * @retval None
 */
void SSD1351_draw_circle(int16_t xc, int16_t yc, uint16_t r, uint16_t colour)
{
	int x = 0, y = r;
	int d = 3 - 2 * r;
	draw_circle(xc, yc, x, y, colour);
	while(y >= x)
	{
		x++;
		if(d > 0)
		{
			y--;
			d = d + 4 * (x - y) + 10;
		}
		else
		{
			d = d + 4 * x + 6;
		}

		draw_circle(xc, yc, x, y, colour);
	}
}

/*
 * @brief Draws a cicle with specified origin and radius into display RAM
 * @param xc: integer for the x origin coordinate
 * @param yc: integer for the y origin coordinate
 * @param colour: colour for the circle
 * @retval None
 */
void SSD1351_draw_filled_circle(int16_t xc, int16_t yc, uint16_t r, uint16_t colour)
{
	int x = 0, y = r;
	int d = 3 - 2 * r;
	draw_filled_circle(xc, yc, x, y, colour);
	while(y >= x)
	{
		x++;
		if(d > 0)
		{
			y--;
			d = d + 4 * (x - y) + 10;
		}
		else
		{
			d = d + 4 * x + 6;
		}

		draw_filled_circle(xc, yc, x, y, colour);
	}
}

/* */
STATIC void SSD1351_write_char(uint16_t colour, const font_t *font, char c)
{
//	if(!font || !font->data || (COLUMNS <= SSD1351_cursor.x + font->width) || (ROWS <= SSD1351_cursor.y + font->height))
//	{
//		return;
//	}

	if(c == '\n')
	{
		SSD1351_cursor.y-=font->height;
		SSD1351_cursor.x=0;
	}
	else
	{
	    if((SSD1351_cursor.x + font->width > COLUMNS))
	    {
	        SSD1351_cursor.y -= font->height;
	        SSD1351_cursor.x = 0;
	    }
		for(int i = 0; i < font->height; i++)
		{
			uint16_t	fd;
			uint16_t	col = ((c - font->first) * font->height);
			if(font->bits == 8)
			{
				const uint8_t	*fdata = (uint8_t *) font->data;
				fd = fdata[col + i];
			}
			else
			{
				const uint16_t	*fdata = (uint16_t *) font->data;
				fd = fdata[col + i];
			}

			for(int j = 0; j < font->width; j++)
			{
				if((fd << j) & (0x1 << (font->bits - 1)))
				{
					SSD1351_write_pixel(SSD1351_cursor.x + j, SSD1351_cursor.y - i, colour);
				}
			}
		}
		SSD1351_cursor.x += font->width;
	}
	return;
}

/* */
STATIC void SSD1351_write_string(uint16_t colour, const font_t *font, char *line)
{
	if(line == NULL)
	{
		return;
	}

	while(*line != 0)
	{
		SSD1351_write_char(colour, font, *line);
		line++;
	}
}

/* */
STATIC void SSD1351_write_int(uint16_t colour, const font_t *font, int8_t n)
{
	char	number[5];
	sprintf(number, "%i", n);
	SSD1351_write_string(colour, font, number);
}

/*
 * @brief Prints a formatted string to the display
 * @param colour: unsigned integer for the colour of the string
 * @param font: structure holding the type of font
 * @param format: formatted string
 */
void SSD1351_printf(uint16_t colour, const font_t *font, const char *format, ...)
{
	if(format == NULL)
	{
		return;
	}

	va_list valist;
	va_start(valist, format);
	while(*format != 0)
	{
		if(*format != '%')
		{
			SSD1351_write_char(colour, font, *format);
			format++;
		}
		else
		{
			format++;
			switch(*format)
			{
			case 's':	SSD1351_write_string(colour, font, va_arg(valist, char *)); break;
			case 'c':	SSD1351_write_char(colour, font, va_arg(valist, int));	//?
				break;
			case 'i':	SSD1351_write_int(colour, font, (int8_t) va_arg(valist, int)); break;
			default:	break;
			}

			format++;
		}
	}
}

/*
 * @brief Sets the printing cursor to a positioin
 * @param x: integer for the x position for the cursor
 * @param y: integer for the y position for the cursor
 */
void SSD1351_set_cursor(uint8_t x, uint8_t y)
{
	SSD1351_cursor.x = x;
	SSD1351_cursor.y = y;
}

uint8_t SSD1351_get_cursor_x(void)
{
    return SSD1351_cursor.x;
}

uint8_t SSD1351_get_cursor_y(void)
{
    return SSD1351_cursor.y;
}

/*
 * @brief Draws a sprite
 * @param sp: pointer to struct holding sprite data
 */
void SSD1351_draw_sprite(int16_t x, int16_t y, sprite *sp)
{
	for(int i = 0; i < sp->width; i++)
	{
		for(int j = 0; j < sp->height; j++)
		{
			uint16_t	colour = color_palette[sp->content[i + (j * sp->width)]];
			if(colour != 0)
			{
				SSD1351_write_pixel(x + i, y + j, colour);
			}
		}
	}
}
