################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_parameter.c \
../VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_reg.c \
../VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_test.c 

C_DEPS += \
./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_parameter.d \
./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_reg.d \
./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_test.d 

OBJS += \
./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_parameter.o \
./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_reg.o \
./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_test.o 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/%.o VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/%.su VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/%.cyclo: ../VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/%.c VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X_1-2e-0-2e-4-2f-LinuxDriverMassMarket_1-2e-0-2e-7-2f-kernel-2f-vl53l0x_test

clean-VL53L0X_1-2e-0-2e-4-2f-LinuxDriverMassMarket_1-2e-0-2e-7-2f-kernel-2f-vl53l0x_test:
	-$(RM) ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_parameter.cyclo ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_parameter.d ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_parameter.o ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_parameter.su ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_reg.cyclo ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_reg.d ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_reg.o ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_reg.su ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_test.cyclo ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_test.d ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_test.o ./VL53L0X_1.0.4/LinuxDriverMassMarket_1.0.7/kernel/vl53l0x_test/vl53l0x_test.su

.PHONY: clean-VL53L0X_1-2e-0-2e-4-2f-LinuxDriverMassMarket_1-2e-0-2e-7-2f-kernel-2f-vl53l0x_test

