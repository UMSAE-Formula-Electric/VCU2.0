
#include "lv_battery_tap.h"
#include "gpio.h"

#include "stm32f4xx.h"
#include "stdint.h"
#include "can.h"
#include "adc.h"

//local constants
#define MAX_ADC_VAL 0x0fff
#define VOLTAGE_RATIO 5.1
const double Vsupply = 3.26; //V determined experimentally

/*
 * lv_battery_voltage
 * This function polls the low voltage battery voltage
 *
 * @return returns the low voltage battery voltage in [Volts]
 */
float lv_battery_voltage(){
	uint32_t myVal = ADC_get_val(ADC_VBATT);


	return VOLTAGE_RATIO*Vsupply*(((float)ADC_get_val(ADC_VBATT)) / MAX_ADC_VAL);
}

/*
 * lv_battery_voltage
 * This function polls the low voltage battery voltage
 *
 * @return returns the low voltage battery voltage in [Volts]
 */
uint16_t lv_battery_voltage_int(){
	return ADC_get_val(ADC_VBATT);
}

/* for 2018 car when no voltage on tap was on the vcu
void lv_battery_handle_voltage_request(){
	uint16_t voltage = 0;
	uint8_t buffer[2] = {0};
	voltage = lv_battery_voltage_int();
	buffer[0] = voltage & 0xff;
	buffer[1] = voltage >> 8;
	send_ACU_mesg_data(	CAN_BATTERY_VOLTAGE_RESPONSE, 2, buffer);

}
*/
