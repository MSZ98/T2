################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f0xx.c 

OBJS += \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f0xx.o 

C_DEPS += \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/main.o: ../Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F030x6 -DDEBUG -c -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Include" -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F030x6 -DDEBUG -c -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Include" -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F030x6 -DDEBUG -c -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Include" -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/system_stm32f0xx.o: ../Src/system_stm32f0xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F030x6 -DDEBUG -c -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Include" -I"C:/ST/STM32CubeIDE_1.3.0/STM32CubeIDE/workspace/T2/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/system_stm32f0xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

