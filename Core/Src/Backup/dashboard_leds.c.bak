#include "dashboard_leds.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "logger.h"


/*
 * ALL GPIO toggle stuff. Every human should be able to understand this
 */
void dash_clear_all_leds() {
	HAL_GPIO_WritePin(TSA_LED1_BLUE_GPIO_Port, TSA_LED1_BLUE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TSA_LED2_GREEN_GPIO_Port, TSA_LED2_GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RTD_LED2_BLUE_GPIO_Port, RTD_LED2_BLUE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RTD_LED1_GREEN_GPIO_Port, RTD_LED1_GREEN_Pin, GPIO_PIN_RESET);
}

void dash_set_all_leds() {
	HAL_GPIO_WritePin(TSA_LED1_BLUE_GPIO_Port, TSA_LED1_BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TSA_LED2_GREEN_GPIO_Port, TSA_LED2_GREEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RTD_LED2_BLUE_GPIO_Port, RTD_LED2_BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RTD_LED1_GREEN_GPIO_Port, RTD_LED1_GREEN_Pin, GPIO_PIN_SET);
}

void dash_set_both_blue() {
	dash_set_tsa_blue();
	dash_set_rtd_blue();
}

/*
 * TSA
 */
void dash_set_tsa_teal() {
	HAL_GPIO_WritePin(TSA_LED1_BLUE_GPIO_Port, TSA_LED1_BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TSA_LED2_GREEN_GPIO_Port, TSA_LED2_GREEN_Pin, GPIO_PIN_SET);
}

void dash_set_tsa_green() {
	HAL_GPIO_WritePin(TSA_LED1_BLUE_GPIO_Port, TSA_LED1_BLUE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TSA_LED2_GREEN_GPIO_Port, TSA_LED2_GREEN_Pin, GPIO_PIN_SET);
}

void dash_set_tsa_blue() {
	HAL_GPIO_WritePin(TSA_LED1_BLUE_GPIO_Port, TSA_LED1_BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TSA_LED2_GREEN_GPIO_Port, TSA_LED2_GREEN_Pin, GPIO_PIN_RESET);
}

void dash_toggle_tsa_blue() {
	HAL_GPIO_TogglePin(TSA_LED1_BLUE_GPIO_Port, TSA_LED1_BLUE_Pin);
}

void dash_toggle_tsa_green() {
	HAL_GPIO_TogglePin(TSA_LED2_GREEN_GPIO_Port, TSA_LED2_GREEN_Pin);
}

void dash_toggle_tsa_teal() {
	HAL_GPIO_TogglePin(TSA_LED2_GREEN_GPIO_Port, TSA_LED2_GREEN_Pin);
	HAL_GPIO_TogglePin(TSA_LED1_BLUE_GPIO_Port, TSA_LED1_BLUE_Pin);
}

/*
 * RTD
 */
void dash_set_rtd_green() {
	HAL_GPIO_WritePin(RTD_LED2_BLUE_GPIO_Port, RTD_LED2_BLUE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RTD_LED1_GREEN_GPIO_Port, RTD_LED1_GREEN_Pin, GPIO_PIN_SET);
}

void dash_set_rtd_blue() {
	HAL_GPIO_WritePin(RTD_LED2_BLUE_GPIO_Port, RTD_LED2_BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RTD_LED1_GREEN_GPIO_Port, RTD_LED1_GREEN_Pin, GPIO_PIN_RESET);
}

void dash_set_rtd_teal() {
	HAL_GPIO_WritePin(RTD_LED2_BLUE_GPIO_Port, RTD_LED2_BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RTD_LED1_GREEN_GPIO_Port, RTD_LED1_GREEN_Pin, GPIO_PIN_SET);
}

void dash_toggle_rtd_green() {
	HAL_GPIO_TogglePin(RTD_LED1_GREEN_GPIO_Port, RTD_LED1_GREEN_Pin);
}

void dash_toggle_rtd_blue() {
	HAL_GPIO_TogglePin(RTD_LED2_BLUE_GPIO_Port, RTD_LED2_BLUE_Pin);
}

void dash_toggle_rtd_teal() {
	HAL_GPIO_TogglePin(RTD_LED2_BLUE_GPIO_Port, RTD_LED2_BLUE_Pin);
	HAL_GPIO_TogglePin(RTD_LED1_GREEN_GPIO_Port, RTD_LED1_GREEN_Pin);
}

void vcu_debug_led(enum debugLed led, bool on){
    GPIO_PinState state = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    uint16_t pin;
    GPIO_TypeDef* port;

    switch(led){
        case REDLED:
            port = VCU_LED_RED_GPIO_Port;
            pin = VCU_LED_RED_Pin;
            break;
        case GREENLED:
            port = VCU_LED_GREEN_GPIO_Port;
            pin = VCU_LED_GREEN_Pin;
            break;
        case BLUELED:
            port = VCU_LED_BLUE_GPIO_Port;
            pin = VCU_LED_BLUE_Pin;
            break;
        default:
            logMessage("Bad argument to debug led", false);
            return;
    }

    HAL_GPIO_WritePin(port, pin, state);
}