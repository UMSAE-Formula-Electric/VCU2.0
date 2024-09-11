/*
 * mc_comms_handler.c
 *
 *  Created on: Apr 27, 2024
 *      Author: tonyz
 */

#include "mc_comms_handler.h"
#include "FreeRTOS.h"
#include "heartbeat.h"
#include "iwdg.h"
#include "can.h"

#define MC_COMMS_DELAY_MS 15

/**
 * @Brief: this function updates the heartbeat task to the presence of the
 * motor controller
 */
void notify_mc_heartbeat_task() {
    TaskHandle_t task = NULL;
    task = get_mc_heartbeat_task_handle();
    if (task != NULL) {
        xTaskNotify(task, 0, eNoAction);
        osThreadFlagsSet(task, 0x01);
    }
}

void StartMcCanCommsTask(void *argument) {
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadExit();
    }

    CAN_RxPacketTypeDef rxPacket;
    osStatus_t isMsgTakenFromQueue;
    uint32_t canId;

    for (;;) {
        kickWatchdogBit(osThreadGetId());

        isMsgTakenFromQueue = osMessageQueueGet(mcCanCommsQueueHandle, &rxPacket, 0, 0);
        if (isMsgTakenFromQueue == osOK) {
            if (rxPacket.rxPacketHeader.IDE == CAN_ID_STD) {
                canId = rxPacket.rxPacketHeader.StdId;
                switch (canId) {
                    case CAN_MC_RX_HIGHSPEED: //High speed message, 333Hz
                        notify_mc_heartbeat_task();
                        mc_process_fast_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_TEMP1_ID: //IGBT temp readings
                        mc_process_temp1_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_TEMP2_ID:
                        mc_process_temp2_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_ANALOG_INPUTS_VOLTAGE:
                        mc_process_analog_inputs_voltage_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_DIGITAL_INPUT_STATUS:
                        mc_process_digital_input_status_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_MOTOR_ID:
                        mc_process_motor_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_CURRENT_ID:
                        mc_process_current_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_VOLT_ID:
                        mc_process_volt_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_FAULT_ID:
                        mc_process_fault_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_INTERNAL_VOLTAGES:
                        mc_process_internal_volt_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_INTERNAL_STATES:
                        mc_process_internal_states_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_TORQUE_TIMER_INFO:
                        mc_process_torque_timer_info_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_MODULATION_INDEX:
                        mc_process_modulation_index_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_FIRMWARE_INFO:
                        mc_process_firmware_info_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_DIAGNOSTIC_DATA:
                        mc_process_diagnostic_data_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_TORQUE_CAPABILITY:
                        mc_process_torque_capability_can(rxPacket.rxPacketData);
                        break;
                    case CAN_MC_RX_TEMP3_ID: //Motor temp reading
                        mc_process_temp3_can(rxPacket.rxPacketData);
                        break;
                    default:
                        break;
                }
            }
        }
        osDelay(MC_COMMS_DELAY_MS);
    }
}
