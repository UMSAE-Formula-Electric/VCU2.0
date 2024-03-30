#ifndef LV_BATTERY_H
#define LV_BATTERY_H
/*
 * Created on June 11 2018
 * Programmed by Martin Rickey
 *
 *
 * This module is used for polling the low voltage battery voltage
 */

void lv_battery_init();
float lv_battery_voltage();
void lv_battery_handle_voltage_request();

#endif
