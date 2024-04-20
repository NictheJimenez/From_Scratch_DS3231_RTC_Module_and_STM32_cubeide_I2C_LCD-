################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/ds3231.c \
../bsp/lcdx.c 

OBJS += \
./bsp/ds3231.o \
./bsp/lcdx.o 

C_DEPS += \
./bsp/ds3231.d \
./bsp/lcdx.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/%.o bsp/%.su bsp/%.cyclo: ../bsp/%.c bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DB_L475E_IOT01A1 -DDEBUG -DSTM32L475VGTx -c -I../Inc -I"G:/STM32CubeIDE/workspaceEmbedded/SPI_I2C_UART_GPIO/stm32l4xx_drivers/bsp" -I"G:/STM32CubeIDE/workspaceEmbedded/SPI_I2C_UART_GPIO/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bsp

clean-bsp:
	-$(RM) ./bsp/ds3231.cyclo ./bsp/ds3231.d ./bsp/ds3231.o ./bsp/ds3231.su ./bsp/lcdx.cyclo ./bsp/lcdx.d ./bsp/lcdx.o ./bsp/lcdx.su

.PHONY: clean-bsp

