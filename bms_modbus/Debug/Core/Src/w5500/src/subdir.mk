################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/w5500/src/socket.c \
../Core/Src/w5500/src/w5500.c \
../Core/Src/w5500/src/wizchip_conf.c 

OBJS += \
./Core/Src/w5500/src/socket.o \
./Core/Src/w5500/src/w5500.o \
./Core/Src/w5500/src/wizchip_conf.o 

C_DEPS += \
./Core/Src/w5500/src/socket.d \
./Core/Src/w5500/src/w5500.d \
./Core/Src/w5500/src/wizchip_conf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/w5500/src/%.o Core/Src/w5500/src/%.su Core/Src/w5500/src/%.cyclo: ../Core/Src/w5500/src/%.c Core/Src/w5500/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/frank/Desktop/bms_modbus/bms_modbus/Core/Src/w5500/inc" -I"C:/Users/frank/Desktop/bms_modbus/bms_modbus/Core/Src/w5500/src" -I../Core/Inc -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-w5500-2f-src

clean-Core-2f-Src-2f-w5500-2f-src:
	-$(RM) ./Core/Src/w5500/src/socket.cyclo ./Core/Src/w5500/src/socket.d ./Core/Src/w5500/src/socket.o ./Core/Src/w5500/src/socket.su ./Core/Src/w5500/src/w5500.cyclo ./Core/Src/w5500/src/w5500.d ./Core/Src/w5500/src/w5500.o ./Core/Src/w5500/src/w5500.su ./Core/Src/w5500/src/wizchip_conf.cyclo ./Core/Src/w5500/src/wizchip_conf.d ./Core/Src/w5500/src/wizchip_conf.o ./Core/Src/w5500/src/wizchip_conf.su

.PHONY: clean-Core-2f-Src-2f-w5500-2f-src

