/*
 * interface.c
 *
 *  Created on: 8 lut 2026
 *      Author: igorp
 */

#include "interface.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "controller.h"
#include <stdio.h>

extern volatile SystemState sys_state;
extern int position;


static char bufor[50];

void Interface_Update(void) {
    ssd1306_Fill(Black);

    sprintf(bufor, "AIM: %d Lx", sys_state.setpoint_lux);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(bufor, Font_11x18, White);

    sprintf(bufor, "ACT: %.1f", sys_state.current_lux);
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString(bufor, Font_11x18, White);

    int pid_pct = sys_state.control_signal / 10;
    sprintf(bufor, "PI:%d%% D:%d%%", pid_pct, position);
    ssd1306_SetCursor(0, 45);
    ssd1306_WriteString(bufor, Font_7x10, White);


    ssd1306_UpdateScreen();
}

void Interface_Init(void) {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

