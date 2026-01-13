################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMI088Middleware.c \
../Core/Src/BMI088driver.c \
../Core/Src/as5047p.c \
../Core/Src/bsp_fdcan.c \
../Core/Src/dma.c \
../Core/Src/fdcan.c \
../Core/Src/gpio.c \
../Core/Src/joint_hw.c \
../Core/Src/joint_mit.c \
../Core/Src/leg_kin.c \
../Core/Src/main.c \
../Core/Src/odrive_can.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/BMI088Middleware.o \
./Core/Src/BMI088driver.o \
./Core/Src/as5047p.o \
./Core/Src/bsp_fdcan.o \
./Core/Src/dma.o \
./Core/Src/fdcan.o \
./Core/Src/gpio.o \
./Core/Src/joint_hw.o \
./Core/Src/joint_mit.o \
./Core/Src/leg_kin.o \
./Core/Src/main.o \
./Core/Src/odrive_can.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/BMI088Middleware.d \
./Core/Src/BMI088driver.d \
./Core/Src/as5047p.d \
./Core/Src/bsp_fdcan.d \
./Core/Src/dma.d \
./Core/Src/fdcan.d \
./Core/Src/gpio.d \
./Core/Src/joint_hw.d \
./Core/Src/joint_mit.d \
./Core/Src/leg_kin.d \
./Core/Src/main.d \
./Core/Src/odrive_can.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMI088Middleware.cyclo ./Core/Src/BMI088Middleware.d ./Core/Src/BMI088Middleware.o ./Core/Src/BMI088Middleware.su ./Core/Src/BMI088driver.cyclo ./Core/Src/BMI088driver.d ./Core/Src/BMI088driver.o ./Core/Src/BMI088driver.su ./Core/Src/as5047p.cyclo ./Core/Src/as5047p.d ./Core/Src/as5047p.o ./Core/Src/as5047p.su ./Core/Src/bsp_fdcan.cyclo ./Core/Src/bsp_fdcan.d ./Core/Src/bsp_fdcan.o ./Core/Src/bsp_fdcan.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/fdcan.cyclo ./Core/Src/fdcan.d ./Core/Src/fdcan.o ./Core/Src/fdcan.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/joint_hw.cyclo ./Core/Src/joint_hw.d ./Core/Src/joint_hw.o ./Core/Src/joint_hw.su ./Core/Src/joint_mit.cyclo ./Core/Src/joint_mit.d ./Core/Src/joint_mit.o ./Core/Src/joint_mit.su ./Core/Src/leg_kin.cyclo ./Core/Src/leg_kin.d ./Core/Src/leg_kin.o ./Core/Src/leg_kin.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/odrive_can.cyclo ./Core/Src/odrive_can.d ./Core/Src/odrive_can.o ./Core/Src/odrive_can.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

