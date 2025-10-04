################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Ext_Kalman_filter/Extkalmanfilter.cpp 

OBJS += \
./Ext_Kalman_filter/Extkalmanfilter.o 

CPP_DEPS += \
./Ext_Kalman_filter/Extkalmanfilter.d 


# Each subdirectory must supply rules for building sources it contributes
Ext_Kalman_filter/%.o Ext_Kalman_filter/%.su Ext_Kalman_filter/%.cyclo: ../Ext_Kalman_filter/%.cpp Ext_Kalman_filter/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I.././ -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Ext_Kalman_filter

clean-Ext_Kalman_filter:
	-$(RM) ./Ext_Kalman_filter/Extkalmanfilter.cyclo ./Ext_Kalman_filter/Extkalmanfilter.d ./Ext_Kalman_filter/Extkalmanfilter.o ./Ext_Kalman_filter/Extkalmanfilter.su

.PHONY: clean-Ext_Kalman_filter

