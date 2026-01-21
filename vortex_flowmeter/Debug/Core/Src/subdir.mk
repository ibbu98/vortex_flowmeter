################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Monitor.c \
../Core/Src/NHD_0216HZ.c \
../Core/Src/Timer0.c \
../Core/Src/UART_poll.c \
../Core/Src/adc_data.c \
../Core/Src/flow_cal.c \
../Core/Src/freq_detect.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/Monitor.o \
./Core/Src/NHD_0216HZ.o \
./Core/Src/Timer0.o \
./Core/Src/UART_poll.o \
./Core/Src/adc_data.o \
./Core/Src/flow_cal.o \
./Core/Src/freq_detect.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/Monitor.d \
./Core/Src/NHD_0216HZ.d \
./Core/Src/Timer0.d \
./Core/Src/UART_poll.d \
./Core/Src/adc_data.d \
./Core/Src/flow_cal.d \
./Core/Src/freq_detect.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Monitor.cyclo ./Core/Src/Monitor.d ./Core/Src/Monitor.o ./Core/Src/Monitor.su ./Core/Src/NHD_0216HZ.cyclo ./Core/Src/NHD_0216HZ.d ./Core/Src/NHD_0216HZ.o ./Core/Src/NHD_0216HZ.su ./Core/Src/Timer0.cyclo ./Core/Src/Timer0.d ./Core/Src/Timer0.o ./Core/Src/Timer0.su ./Core/Src/UART_poll.cyclo ./Core/Src/UART_poll.d ./Core/Src/UART_poll.o ./Core/Src/UART_poll.su ./Core/Src/adc_data.cyclo ./Core/Src/adc_data.d ./Core/Src/adc_data.o ./Core/Src/adc_data.su ./Core/Src/flow_cal.cyclo ./Core/Src/flow_cal.d ./Core/Src/flow_cal.o ./Core/Src/flow_cal.su ./Core/Src/freq_detect.cyclo ./Core/Src/freq_detect.d ./Core/Src/freq_detect.o ./Core/Src/freq_detect.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

