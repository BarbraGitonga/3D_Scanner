################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X_1.0.4/ApiExample/examples/src/get_nucleo_serial_comm.c \
../VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_ContinuousRanging_Example.c \
../VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Example.c \
../VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Accuracy_Example.c \
../VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Speed_Example.c \
../VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Long_Range_Example.c 

C_DEPS += \
./VL53L0X_1.0.4/ApiExample/examples/src/get_nucleo_serial_comm.d \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_ContinuousRanging_Example.d \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Example.d \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Accuracy_Example.d \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Speed_Example.d \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Long_Range_Example.d 

OBJS += \
./VL53L0X_1.0.4/ApiExample/examples/src/get_nucleo_serial_comm.o \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_ContinuousRanging_Example.o \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Example.o \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Accuracy_Example.o \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Speed_Example.o \
./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Long_Range_Example.o 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X_1.0.4/ApiExample/examples/src/%.o VL53L0X_1.0.4/ApiExample/examples/src/%.su VL53L0X_1.0.4/ApiExample/examples/src/%.cyclo: ../VL53L0X_1.0.4/ApiExample/examples/src/%.c VL53L0X_1.0.4/ApiExample/examples/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X_1-2e-0-2e-4-2f-ApiExample-2f-examples-2f-src

clean-VL53L0X_1-2e-0-2e-4-2f-ApiExample-2f-examples-2f-src:
	-$(RM) ./VL53L0X_1.0.4/ApiExample/examples/src/get_nucleo_serial_comm.cyclo ./VL53L0X_1.0.4/ApiExample/examples/src/get_nucleo_serial_comm.d ./VL53L0X_1.0.4/ApiExample/examples/src/get_nucleo_serial_comm.o ./VL53L0X_1.0.4/ApiExample/examples/src/get_nucleo_serial_comm.su ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_ContinuousRanging_Example.cyclo ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_ContinuousRanging_Example.d ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_ContinuousRanging_Example.o ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_ContinuousRanging_Example.su ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Example.cyclo ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Example.d ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Example.o ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Example.su ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Accuracy_Example.cyclo ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Accuracy_Example.d ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Accuracy_Example.o ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Accuracy_Example.su ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Speed_Example.cyclo ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Speed_Example.d ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Speed_Example.o ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_High_Speed_Example.su ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Long_Range_Example.cyclo ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Long_Range_Example.d ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Long_Range_Example.o ./VL53L0X_1.0.4/ApiExample/examples/src/vl53l0x_SingleRanging_Long_Range_Example.su

.PHONY: clean-VL53L0X_1-2e-0-2e-4-2f-ApiExample-2f-examples-2f-src

