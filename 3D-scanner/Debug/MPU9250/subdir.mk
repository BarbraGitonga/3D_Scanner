################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MPU9250/MPU9250.cpp \
../MPU9250/MPU9250_test.cpp 

OBJS += \
./MPU9250/MPU9250.o \
./MPU9250/MPU9250_test.o 

CPP_DEPS += \
./MPU9250/MPU9250.d \
./MPU9250/MPU9250_test.d 


# Each subdirectory must supply rules for building sources it contributes
MPU9250/%.o MPU9250/%.su MPU9250/%.cyclo: ../MPU9250/%.cpp MPU9250/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I.././ -I"C:/Users/lenovo/Desktop/3D_Scanner/3D_Scanner_cpp/DISCO/3D-scanner/Ext_Kalman_filter" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MPU9250

clean-MPU9250:
	-$(RM) ./MPU9250/MPU9250.cyclo ./MPU9250/MPU9250.d ./MPU9250/MPU9250.o ./MPU9250/MPU9250.su ./MPU9250/MPU9250_test.cyclo ./MPU9250/MPU9250_test.d ./MPU9250/MPU9250_test.o ./MPU9250/MPU9250_test.su

.PHONY: clean-MPU9250

