#include <stdbool.h>

#ifndef DASHBOARD_LEDS_H
#define DASHBAORD_LEDS_H

enum debugLed{
    REDLED=0,
    GREENLED,
    BLUELED
};

void dash_leds_init();
void dash_clear_all_leds();
void dash_set_all_leds();
void dash_set_both_blue();


void dash_set_tsa_green();
void dash_set_tsa_blue();
void dash_set_tsa_teal();

void dash_set_rtd_green();
void dash_set_rtd_blue();
void dash_set_rtd_teal();

void dash_toggle_tsa_blue();
void dash_toggle_tsa_green();
void dash_toggle_tsa_teal();
void dash_flash_tsa_teal();

void dash_toggle_rtd_green();
void dash_toggle_rtd_blue();
void dash_toggle_rtd_teal();
void dash_flash_rtd_teal();

void dash_fail_pulse();

void vcu_debug_led_init();
void vcu_debug_led(enum debugLed led, bool on);

#endif
