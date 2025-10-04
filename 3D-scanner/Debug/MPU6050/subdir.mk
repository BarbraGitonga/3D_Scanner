################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MPU6050/MPU6050.cpp \
../MPU6050/MPU6050_test.cpp 

OBJS += \
./MPU6050/MPU6050.o \
./MPU6050/MPU6050_test.o 

CPP_DEPS += \
./MPU6050/MPU6050.d \
./MPU6050/MPU6050_test.d 


# Each subdirectory must supply rules for building sources it contributes
MPU6050/%.o MPU6050/%.su MPU6050/%.cyclo: ../MPU6050/%.cpp MPU6050/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I.././ -I"C:/Users/lenovo/Desktop/3D_Scanner/3D-scanner/Ext_Kalman_filter" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MPU6050

clean-MPU6050:
	-$(RM) ./MPU6050/MPU6050.cyclo ./MPU6050/MPU6050.d ./MPU6050/MPU6050.o ./MPU6050/MPU6050.su ./MPU6050/MPU6050_test.cyclo ./MPU6050/MPU6050_test.d ./MPU6050/MPU6050_test.o ./MPU6050/MPU6050_test.su

.PHONY: clean-MPU6050

