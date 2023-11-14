################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32L0xx_Nucleo/stm32l0xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32L0xx_Nucleo/stm32l0xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32L0xx_Nucleo/stm32l0xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32L0xx_Nucleo/stm32l0xx_nucleo.o: ../Drivers/BSP/STM32L0xx_Nucleo/stm32l0xx_nucleo.c Drivers/BSP/STM32L0xx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L051xx -DUSE_STM32L0XX_NUCLEO '-DLIGHT_CODE=1' '-DSX1262DVK1CAS=1' -c -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/LoRaWAN/Target" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/LoRaWAN/App" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Core/Inc" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Utilities/misc" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Utilities/timer" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Utilities/trace/adv_trace" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Utilities/lpm/tiny_lpm" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Utilities/sequencer" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Drivers/CMSIS/Include" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/SubGHz_Phy" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/LoRaWAN/Crypto" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/LoRaWAN/Mac" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/LoRaWAN/Mac/Region" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/LoRaWAN/Utilities" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/LoRaWAN/LmHandler" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/LoRaWAN/LmHandler/Packages" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Drivers/BSP/STM32L0xx_Nucleo" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Drivers/STM32L0xx_HAL_Driver/Inc" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Middlewares/Third_Party/SubGHz_Phy/sx126x" -I"C:/Users/JESUS TE AMA/STM32CubeIDE/workspace_1.6.0/TX_Estacionamento/Drivers/BSP/SX1262DVK1CAS" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32L0xx_Nucleo/stm32l0xx_nucleo.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

