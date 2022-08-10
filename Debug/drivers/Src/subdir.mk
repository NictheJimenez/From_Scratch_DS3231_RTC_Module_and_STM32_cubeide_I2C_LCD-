################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32l475xx_gpio_driver.c \
../drivers/Src/stm32l475xx_i2c_driver.c \
../drivers/Src/stm32l475xx_spi_drivers.c 

OBJS += \
./drivers/Src/stm32l475xx_gpio_driver.o \
./drivers/Src/stm32l475xx_i2c_driver.o \
./drivers/Src/stm32l475xx_spi_drivers.o 

C_DEPS += \
./drivers/Src/stm32l475xx_gpio_driver.d \
./drivers/Src/stm32l475xx_i2c_driver.d \
./drivers/Src/stm32l475xx_spi_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32l475xx_gpio_driver.o: ../drivers/Src/stm32l475xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DB_L475E_IOT01A1 -DDEBUG -DSTM32L475VGTx -c -I../Inc -I"C:/Users/nicth/STM32CubeIDE/workspaceEmbedded/SPI_I2C_UART_GPIO/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32l475xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32l475xx_i2c_driver.o: ../drivers/Src/stm32l475xx_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DB_L475E_IOT01A1 -DDEBUG -DSTM32L475VGTx -c -I../Inc -I"C:/Users/nicth/STM32CubeIDE/workspaceEmbedded/SPI_I2C_UART_GPIO/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32l475xx_i2c_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32l475xx_spi_drivers.o: ../drivers/Src/stm32l475xx_spi_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4 -DSTM32 -DB_L475E_IOT01A1 -DDEBUG -DSTM32L475VGTx -c -I../Inc -I"C:/Users/nicth/STM32CubeIDE/workspaceEmbedded/SPI_I2C_UART_GPIO/stm32l4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32l475xx_spi_drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

