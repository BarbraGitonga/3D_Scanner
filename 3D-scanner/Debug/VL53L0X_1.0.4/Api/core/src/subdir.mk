################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X_1.0.4/Api/core/src/vl53l0x_api.c \
../VL53L0X_1.0.4/Api/core/src/vl53l0x_api_calibration.c \
../VL53L0X_1.0.4/Api/core/src/vl53l0x_api_core.c \
../VL53L0X_1.0.4/Api/core/src/vl53l0x_api_ranging.c \
../VL53L0X_1.0.4/Api/core/src/vl53l0x_api_strings.c 

C_DEPS += \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api.d \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_calibration.d \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_core.d \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_ranging.d \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_strings.d 

OBJS += \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api.o \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_calibration.o \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_core.o \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_ranging.o \
./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_strings.o 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X_1.0.4/Api/core/src/%.o VL53L0X_1.0.4/Api/core/src/%.su VL53L0X_1.0.4/Api/core/src/%.cyclo: ../VL53L0X_1.0.4/Api/core/src/%.c VL53L0X_1.0.4/Api/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X_1-2e-0-2e-4-2f-Api-2f-core-2f-src

clean-VL53L0X_1-2e-0-2e-4-2f-Api-2f-core-2f-src:
	-$(RM) ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api.cyclo ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api.d ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api.o ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api.su ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_calibration.cyclo ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_calibration.d ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_calibration.o ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_calibration.su ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_core.cyclo ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_core.d ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_core.o ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_core.su ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_ranging.cyclo ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_ranging.d ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_ranging.o ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_ranging.su ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_strings.cyclo ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_strings.d ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_strings.o ./VL53L0X_1.0.4/Api/core/src/vl53l0x_api_strings.su

.PHONY: clean-VL53L0X_1-2e-0-2e-4-2f-Api-2f-core-2f-src

