################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/monitor.c \
../Core/Src/stm32f4xx_hal_pcd.c \
../Core/Src/stm32f4xx_hal_pcd_ex.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/stm32f4xx_ll_usb.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/usb_device.c \
../Core/Src/usbd_cdc.c \
../Core/Src/usbd_cdc_if.c \
../Core/Src/usbd_conf.c \
../Core/Src/usbd_core.c \
../Core/Src/usbd_ctlreq.c \
../Core/Src/usbd_desc.c \
../Core/Src/usbd_ioreq.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/monitor.o \
./Core/Src/stm32f4xx_hal_pcd.o \
./Core/Src/stm32f4xx_hal_pcd_ex.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/stm32f4xx_ll_usb.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/usb_device.o \
./Core/Src/usbd_cdc.o \
./Core/Src/usbd_cdc_if.o \
./Core/Src/usbd_conf.o \
./Core/Src/usbd_core.o \
./Core/Src/usbd_ctlreq.o \
./Core/Src/usbd_desc.o \
./Core/Src/usbd_ioreq.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/monitor.d \
./Core/Src/stm32f4xx_hal_pcd.d \
./Core/Src/stm32f4xx_hal_pcd_ex.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/stm32f4xx_ll_usb.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/usb_device.d \
./Core/Src/usbd_cdc.d \
./Core/Src/usbd_cdc_if.d \
./Core/Src/usbd_conf.d \
./Core/Src/usbd_core.d \
./Core/Src/usbd_ctlreq.d \
./Core/Src/usbd_desc.d \
./Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DHAL_PCD_MODULE_ENABLED -DDEBUG -DARM_MATH_CM4 -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/monitor.cyclo ./Core/Src/monitor.d ./Core/Src/monitor.o ./Core/Src/monitor.su ./Core/Src/stm32f4xx_hal_pcd.cyclo ./Core/Src/stm32f4xx_hal_pcd.d ./Core/Src/stm32f4xx_hal_pcd.o ./Core/Src/stm32f4xx_hal_pcd.su ./Core/Src/stm32f4xx_hal_pcd_ex.cyclo ./Core/Src/stm32f4xx_hal_pcd_ex.d ./Core/Src/stm32f4xx_hal_pcd_ex.o ./Core/Src/stm32f4xx_hal_pcd_ex.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/stm32f4xx_ll_usb.cyclo ./Core/Src/stm32f4xx_ll_usb.d ./Core/Src/stm32f4xx_ll_usb.o ./Core/Src/stm32f4xx_ll_usb.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/usb_device.cyclo ./Core/Src/usb_device.d ./Core/Src/usb_device.o ./Core/Src/usb_device.su ./Core/Src/usbd_cdc.cyclo ./Core/Src/usbd_cdc.d ./Core/Src/usbd_cdc.o ./Core/Src/usbd_cdc.su ./Core/Src/usbd_cdc_if.cyclo ./Core/Src/usbd_cdc_if.d ./Core/Src/usbd_cdc_if.o ./Core/Src/usbd_cdc_if.su ./Core/Src/usbd_conf.cyclo ./Core/Src/usbd_conf.d ./Core/Src/usbd_conf.o ./Core/Src/usbd_conf.su ./Core/Src/usbd_core.cyclo ./Core/Src/usbd_core.d ./Core/Src/usbd_core.o ./Core/Src/usbd_core.su ./Core/Src/usbd_ctlreq.cyclo ./Core/Src/usbd_ctlreq.d ./Core/Src/usbd_ctlreq.o ./Core/Src/usbd_ctlreq.su ./Core/Src/usbd_desc.cyclo ./Core/Src/usbd_desc.d ./Core/Src/usbd_desc.o ./Core/Src/usbd_desc.su ./Core/Src/usbd_ioreq.cyclo ./Core/Src/usbd_ioreq.d ./Core/Src/usbd_ioreq.o ./Core/Src/usbd_ioreq.su

.PHONY: clean-Core-2f-Src

