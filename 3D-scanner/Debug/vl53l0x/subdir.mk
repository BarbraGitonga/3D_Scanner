################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vl53l0x/vl53l0x.c 

C_DEPS += \
./vl53l0x/vl53l0x.d 

OBJS += \
./vl53l0x/vl53l0x.o 


# Each subdirectory must supply rules for building sources it contributes
vl53l0x/%.o vl53l0x/%.su vl53l0x/%.cyclo: ../vl53l0x/%.c vl53l0x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-vl53l0x

clean-vl53l0x:
	-$(RM) ./vl53l0x/vl53l0x.cyclo ./vl53l0x/vl53l0x.d ./vl53l0x/vl53l0x.o ./vl53l0x/vl53l0x.su

.PHONY: clean-vl53l0x

