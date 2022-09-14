################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/stm32f411xx_gpio.c 

OBJS += \
./driver/Src/stm32f411xx_gpio.o 

C_DEPS += \
./driver/Src/stm32f411xx_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/%.o driver/Src/%.su: ../driver/Src/%.c driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411CEUx -c -I"D:/Development/STM32/GPIO_driver_rev/driver/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-driver-2f-Src

clean-driver-2f-Src:
	-$(RM) ./driver/Src/stm32f411xx_gpio.d ./driver/Src/stm32f411xx_gpio.o ./driver/Src/stm32f411xx_gpio.su

.PHONY: clean-driver-2f-Src

