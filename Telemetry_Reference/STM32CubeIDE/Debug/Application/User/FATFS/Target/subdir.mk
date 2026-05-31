################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Programming_Folder/Stm32/Telemetry_Board/Telemetry_Reference/FATFS/Target/user_diskio.c 

OBJS += \
./Application/User/FATFS/Target/user_diskio.o 

C_DEPS += \
./Application/User/FATFS/Target/user_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/FATFS/Target/user_diskio.o: E:/Programming_Folder/Stm32/Telemetry_Board/Telemetry_Reference/FATFS/Target/user_diskio.c Application/User/FATFS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../../FATFS/Target -I../../FATFS/App -I../../Core/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FatFs/src -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-FATFS-2f-Target

clean-Application-2f-User-2f-FATFS-2f-Target:
	-$(RM) ./Application/User/FATFS/Target/user_diskio.cyclo ./Application/User/FATFS/Target/user_diskio.d ./Application/User/FATFS/Target/user_diskio.o ./Application/User/FATFS/Target/user_diskio.su

.PHONY: clean-Application-2f-User-2f-FATFS-2f-Target

