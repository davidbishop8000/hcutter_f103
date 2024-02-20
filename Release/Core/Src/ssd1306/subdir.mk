################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ssd1306/ssd1306.c \
../Core/Src/ssd1306/ssd1306_fonts.c \
../Core/Src/ssd1306/ssd1306_tests.c 

C_DEPS += \
./Core/Src/ssd1306/ssd1306.d \
./Core/Src/ssd1306/ssd1306_fonts.d \
./Core/Src/ssd1306/ssd1306_tests.d 

OBJS += \
./Core/Src/ssd1306/ssd1306.o \
./Core/Src/ssd1306/ssd1306_fonts.o \
./Core/Src/ssd1306/ssd1306_tests.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ssd1306/%.o Core/Src/ssd1306/%.su: ../Core/Src/ssd1306/%.c Core/Src/ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ssd1306

clean-Core-2f-Src-2f-ssd1306:
	-$(RM) ./Core/Src/ssd1306/ssd1306.d ./Core/Src/ssd1306/ssd1306.o ./Core/Src/ssd1306/ssd1306.su ./Core/Src/ssd1306/ssd1306_fonts.d ./Core/Src/ssd1306/ssd1306_fonts.o ./Core/Src/ssd1306/ssd1306_fonts.su ./Core/Src/ssd1306/ssd1306_tests.d ./Core/Src/ssd1306/ssd1306_tests.o ./Core/Src/ssd1306/ssd1306_tests.su

.PHONY: clean-Core-2f-Src-2f-ssd1306

