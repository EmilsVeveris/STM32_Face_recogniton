################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/App/appLcdSpeedTest.c \
../Core/Src/App/beer_60x100_16.c 

OBJS += \
./Core/Src/App/appLcdSpeedTest.o \
./Core/Src/App/beer_60x100_16.o 

C_DEPS += \
./Core/Src/App/appLcdSpeedTest.d \
./Core/Src/App/beer_60x100_16.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/App/appLcdSpeedTest.o: ../Core/Src/App/appLcdSpeedTest.c Core/Src/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/emils/OneDrive/Desktop/Augstskola/3.kurss/2.Semestris/Iegultas_operetajsistemas1/Kursa_Darbs_V1.0/Core/camera" -I"C:/Users/emils/OneDrive/Desktop/Augstskola/3.kurss/2.Semestris/Iegultas_operetajsistemas1/Kursa_Darbs_V1.0/Core/Src/Lcd" -I"C:/Users/emils/OneDrive/Desktop/Augstskola/3.kurss/2.Semestris/Iegultas_operetajsistemas1/Kursa_Darbs_V1.0/Core/Src/App" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/App/appLcdSpeedTest.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/App/beer_60x100_16.o: ../Core/Src/App/beer_60x100_16.c Core/Src/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/emils/OneDrive/Desktop/Augstskola/3.kurss/2.Semestris/Iegultas_operetajsistemas1/Kursa_Darbs_V1.0/Core/camera" -I"C:/Users/emils/OneDrive/Desktop/Augstskola/3.kurss/2.Semestris/Iegultas_operetajsistemas1/Kursa_Darbs_V1.0/Core/Src/Lcd" -I"C:/Users/emils/OneDrive/Desktop/Augstskola/3.kurss/2.Semestris/Iegultas_operetajsistemas1/Kursa_Darbs_V1.0/Core/Src/App" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/App/beer_60x100_16.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

