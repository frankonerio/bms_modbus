################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/w5500/W5500/w5500.c 

OBJS += \
./Core/Src/w5500/W5500/w5500.o 

C_DEPS += \
./Core/Src/w5500/W5500/w5500.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/w5500/W5500/%.o Core/Src/w5500/W5500/%.su Core/Src/w5500/W5500/%.cyclo: ../Core/Src/w5500/W5500/%.c Core/Src/w5500/W5500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/frank/Desktop/bms_modbus/bms_modbus/Core/Src/w5500" -I"C:/Users/frank/Desktop/bms_modbus/bms_modbus/Core/Src/w5500/W5500" -I../Core/Inc -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/frank/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-w5500-2f-W5500

clean-Core-2f-Src-2f-w5500-2f-W5500:
	-$(RM) ./Core/Src/w5500/W5500/w5500.cyclo ./Core/Src/w5500/W5500/w5500.d ./Core/Src/w5500/W5500/w5500.o ./Core/Src/w5500/W5500/w5500.su

.PHONY: clean-Core-2f-Src-2f-w5500-2f-W5500

