################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407xx_ADC_driver.c \
../drivers/Src/stm32f407xx_I2C_driver.c \
../drivers/Src/stm32f407xx_gpio_driver.c \
../drivers/Src/stm32f407xx_spi_driver.c \
../drivers/Src/stm32f407xx_timer.c 

OBJS += \
./drivers/Src/stm32f407xx_ADC_driver.o \
./drivers/Src/stm32f407xx_I2C_driver.o \
./drivers/Src/stm32f407xx_gpio_driver.o \
./drivers/Src/stm32f407xx_spi_driver.o \
./drivers/Src/stm32f407xx_timer.o 

C_DEPS += \
./drivers/Src/stm32f407xx_ADC_driver.d \
./drivers/Src/stm32f407xx_I2C_driver.d \
./drivers/Src/stm32f407xx_gpio_driver.d \
./drivers/Src/stm32f407xx_spi_driver.d \
./drivers/Src/stm32f407xx_timer.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"E:/Embedded-C/My_workspace/stm32f407_drivers/stm32f407_drivers/stm32f407_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

