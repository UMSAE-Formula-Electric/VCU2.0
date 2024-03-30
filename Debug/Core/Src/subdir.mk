################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ACB_comms_handler.c \
../Core/Src/adc.c \
../Core/Src/apps_brake.c \
../Core/Src/bt_protocol.c \
../Core/Src/can.c \
../Core/Src/car_state.c \
../Core/Src/dashboard_leds.c \
../Core/Src/dashboard_mgmt.c \
../Core/Src/dma.c \
../Core/Src/error_handler.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/heartbeat.c \
../Core/Src/iwdg.c \
../Core/Src/logger.c \
../Core/Src/lv_battery_tap.c \
../Core/Src/main.c \
../Core/Src/motor_controller_can.c \
../Core/Src/pedal_encoder.c \
../Core/Src/startup_condition.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/vcu_startup.c 

OBJS += \
./Core/Src/ACB_comms_handler.o \
./Core/Src/adc.o \
./Core/Src/apps_brake.o \
./Core/Src/bt_protocol.o \
./Core/Src/can.o \
./Core/Src/car_state.o \
./Core/Src/dashboard_leds.o \
./Core/Src/dashboard_mgmt.o \
./Core/Src/dma.o \
./Core/Src/error_handler.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/heartbeat.o \
./Core/Src/iwdg.o \
./Core/Src/logger.o \
./Core/Src/lv_battery_tap.o \
./Core/Src/main.o \
./Core/Src/motor_controller_can.o \
./Core/Src/pedal_encoder.o \
./Core/Src/startup_condition.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/vcu_startup.o 

C_DEPS += \
./Core/Src/ACB_comms_handler.d \
./Core/Src/adc.d \
./Core/Src/apps_brake.d \
./Core/Src/bt_protocol.d \
./Core/Src/can.d \
./Core/Src/car_state.d \
./Core/Src/dashboard_leds.d \
./Core/Src/dashboard_mgmt.d \
./Core/Src/dma.d \
./Core/Src/error_handler.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/heartbeat.d \
./Core/Src/iwdg.d \
./Core/Src/logger.d \
./Core/Src/lv_battery_tap.d \
./Core/Src/main.d \
./Core/Src/motor_controller_can.d \
./Core/Src/pedal_encoder.d \
./Core/Src/startup_condition.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/vcu_startup.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ACB_comms_handler.cyclo ./Core/Src/ACB_comms_handler.d ./Core/Src/ACB_comms_handler.o ./Core/Src/ACB_comms_handler.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/apps_brake.cyclo ./Core/Src/apps_brake.d ./Core/Src/apps_brake.o ./Core/Src/apps_brake.su ./Core/Src/bt_protocol.cyclo ./Core/Src/bt_protocol.d ./Core/Src/bt_protocol.o ./Core/Src/bt_protocol.su ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/car_state.cyclo ./Core/Src/car_state.d ./Core/Src/car_state.o ./Core/Src/car_state.su ./Core/Src/dashboard_leds.cyclo ./Core/Src/dashboard_leds.d ./Core/Src/dashboard_leds.o ./Core/Src/dashboard_leds.su ./Core/Src/dashboard_mgmt.cyclo ./Core/Src/dashboard_mgmt.d ./Core/Src/dashboard_mgmt.o ./Core/Src/dashboard_mgmt.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/error_handler.cyclo ./Core/Src/error_handler.d ./Core/Src/error_handler.o ./Core/Src/error_handler.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/heartbeat.cyclo ./Core/Src/heartbeat.d ./Core/Src/heartbeat.o ./Core/Src/heartbeat.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/logger.cyclo ./Core/Src/logger.d ./Core/Src/logger.o ./Core/Src/logger.su ./Core/Src/lv_battery_tap.cyclo ./Core/Src/lv_battery_tap.d ./Core/Src/lv_battery_tap.o ./Core/Src/lv_battery_tap.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_controller_can.cyclo ./Core/Src/motor_controller_can.d ./Core/Src/motor_controller_can.o ./Core/Src/motor_controller_can.su ./Core/Src/pedal_encoder.cyclo ./Core/Src/pedal_encoder.d ./Core/Src/pedal_encoder.o ./Core/Src/pedal_encoder.su ./Core/Src/startup_condition.cyclo ./Core/Src/startup_condition.d ./Core/Src/startup_condition.o ./Core/Src/startup_condition.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/vcu_startup.cyclo ./Core/Src/vcu_startup.d ./Core/Src/vcu_startup.o ./Core/Src/vcu_startup.su

.PHONY: clean-Core-2f-Src

