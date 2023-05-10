################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DHT.c \
../Core/Src/dhtxx.c \
../Core/Src/dwt.c \
../Core/Src/font12epd.c \
../Core/Src/font16epd.c \
../Core/Src/font20epd.c \
../Core/Src/font8epd.c \
../Core/Src/gde021a1.c \
../Core/Src/main.c \
../Core/Src/stm32l0538_discovery_epd.c \
../Core/Src/stm32l0xx_hal_msp.c \
../Core/Src/stm32l0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l0xx.c 

OBJS += \
./Core/Src/DHT.o \
./Core/Src/dhtxx.o \
./Core/Src/dwt.o \
./Core/Src/font12epd.o \
./Core/Src/font16epd.o \
./Core/Src/font20epd.o \
./Core/Src/font8epd.o \
./Core/Src/gde021a1.o \
./Core/Src/main.o \
./Core/Src/stm32l0538_discovery_epd.o \
./Core/Src/stm32l0xx_hal_msp.o \
./Core/Src/stm32l0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l0xx.o 

C_DEPS += \
./Core/Src/DHT.d \
./Core/Src/dhtxx.d \
./Core/Src/dwt.d \
./Core/Src/font12epd.d \
./Core/Src/font16epd.d \
./Core/Src/font20epd.d \
./Core/Src/font8epd.d \
./Core/Src/gde021a1.d \
./Core/Src/main.d \
./Core/Src/stm32l0538_discovery_epd.d \
./Core/Src/stm32l0xx_hal_msp.d \
./Core/Src/stm32l0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DHT.cyclo ./Core/Src/DHT.d ./Core/Src/DHT.o ./Core/Src/DHT.su ./Core/Src/dhtxx.cyclo ./Core/Src/dhtxx.d ./Core/Src/dhtxx.o ./Core/Src/dhtxx.su ./Core/Src/dwt.cyclo ./Core/Src/dwt.d ./Core/Src/dwt.o ./Core/Src/dwt.su ./Core/Src/font12epd.cyclo ./Core/Src/font12epd.d ./Core/Src/font12epd.o ./Core/Src/font12epd.su ./Core/Src/font16epd.cyclo ./Core/Src/font16epd.d ./Core/Src/font16epd.o ./Core/Src/font16epd.su ./Core/Src/font20epd.cyclo ./Core/Src/font20epd.d ./Core/Src/font20epd.o ./Core/Src/font20epd.su ./Core/Src/font8epd.cyclo ./Core/Src/font8epd.d ./Core/Src/font8epd.o ./Core/Src/font8epd.su ./Core/Src/gde021a1.cyclo ./Core/Src/gde021a1.d ./Core/Src/gde021a1.o ./Core/Src/gde021a1.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l0538_discovery_epd.cyclo ./Core/Src/stm32l0538_discovery_epd.d ./Core/Src/stm32l0538_discovery_epd.o ./Core/Src/stm32l0538_discovery_epd.su ./Core/Src/stm32l0xx_hal_msp.cyclo ./Core/Src/stm32l0xx_hal_msp.d ./Core/Src/stm32l0xx_hal_msp.o ./Core/Src/stm32l0xx_hal_msp.su ./Core/Src/stm32l0xx_it.cyclo ./Core/Src/stm32l0xx_it.d ./Core/Src/stm32l0xx_it.o ./Core/Src/stm32l0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l0xx.cyclo ./Core/Src/system_stm32l0xx.d ./Core/Src/system_stm32l0xx.o ./Core/Src/system_stm32l0xx.su

.PHONY: clean-Core-2f-Src

