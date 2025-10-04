################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../KalmanFilter/KalmanFilter.cpp 

OBJS += \
./KalmanFilter/KalmanFilter.o 

CPP_DEPS += \
./KalmanFilter/KalmanFilter.d 


# Each subdirectory must supply rules for building sources it contributes
KalmanFilter/%.o KalmanFilter/%.su KalmanFilter/%.cyclo: ../KalmanFilter/%.cpp KalmanFilter/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I.././ -I"C:/Users/lenovo/Desktop/3D_Scanner/3D_Scanner_cpp/DISCO/3D-scanner/Ext_Kalman_filter" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-KalmanFilter

clean-KalmanFilter:
	-$(RM) ./KalmanFilter/KalmanFilter.cyclo ./KalmanFilter/KalmanFilter.d ./KalmanFilter/KalmanFilter.o ./KalmanFilter/KalmanFilter.su

.PHONY: clean-KalmanFilter

