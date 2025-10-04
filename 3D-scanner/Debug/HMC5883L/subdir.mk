################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../HMC5883L/HMC5883L.cpp 

OBJS += \
./HMC5883L/HMC5883L.o 

CPP_DEPS += \
./HMC5883L/HMC5883L.d 


# Each subdirectory must supply rules for building sources it contributes
HMC5883L/%.o HMC5883L/%.su HMC5883L/%.cyclo: ../HMC5883L/%.cpp HMC5883L/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I.././ -I"C:/Users/lenovo/Desktop/3D_Scanner/3D_Scanner_cpp/DISCO/3D-scanner/Ext_Kalman_filter" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-HMC5883L

clean-HMC5883L:
	-$(RM) ./HMC5883L/HMC5883L.cyclo ./HMC5883L/HMC5883L.d ./HMC5883L/HMC5883L.o ./HMC5883L/HMC5883L.su

.PHONY: clean-HMC5883L

