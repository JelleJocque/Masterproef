/*
 * OLED.c
 *
 *  Created on: Mar 15, 2020
 *      Author: jelle
 */
#include "OLED.h"

/* Variables -------------------------------------------------------------------*/

/* Functions -------------------------------------------------------------------*/
void OLED_init(void)
{
	SSD1306_Init();
}

void OLED_print_text(char command[], uint8_t x, uint8_t y)
{
	SSD1306_GotoXY(x,y);
	SSD1306_Puts(command, &Font_7x10, SSD1306_COLOR_WHITE);
}

void OLED_print_title(char command[], uint8_t x, uint8_t y)
{
	SSD1306_GotoXY(x,y);
	SSD1306_Puts(command, &Font_11x18, SSD1306_COLOR_WHITE);
}

void OLED_print_variable(char command[], uint32_t value, uint8_t x, uint8_t y)
{
	char stringValue[10];
	sprintf(stringValue, "%d", value);
	SSD1306_GotoXY(x,y);
	SSD1306_Puts(command, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(x+(strlen(command)*7),y);
	SSD1306_Puts(stringValue, &Font_7x10, SSD1306_COLOR_WHITE);
}

void OLED_clear_screen(void)
{
	SSD1306_Fill(SSD1306_COLOR_BLACK);
}

void OLED_update(void)
{
	SSD1306_UpdateScreen();
}
