################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/Components/exc7200/exc7200.c 

OBJS += \
./Utilities/Components/exc7200/exc7200.o 

C_DEPS += \
./Utilities/Components/exc7200/exc7200.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/Components/exc7200/%.o: ../Utilities/Components/exc7200/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ili9325" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/s25fl512s" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/cs43l22" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ili9341" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ampire480272" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/n25q512a" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/s5k5cag" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/mfxstm32l152" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/CMSIS/device" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/n25q128a" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ts3510" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/st7735" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/HAL_Driver/Inc/Legacy" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/lis302dl" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/otm8009a" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/stmpe1600" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ov2640" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/Common" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/l3gd20" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/HAL_Driver/Inc" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/stmpe811" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/lis3dsh" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/wm8994" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/n25q256a" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ls016b8uy" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/inc" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ft6x06" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/STM32F4-Discovery" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/exc7200" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/st7789h2" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/ampire640480" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/Utilities/Components/lsm303dlhc" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/CMSIS/core" -I/home/phong/Documents/workspace/workspace_sw4stm32/stm32f4_hal_ros/ros_lib -I"/home/phong/Documents/thesis/stm32f4_hal_ros/components/driver/include" -I"/home/phong/Documents/thesis/stm32f4_hal_ros/components/mpu6050/include" -I/home/phong/Documents/thesis/stm32f4_hal_ros/components/mpu6050/include -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


