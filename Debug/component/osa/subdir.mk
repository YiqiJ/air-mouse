################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/osa/fsl_os_abstraction_bm.c 

OBJS += \
./component/osa/fsl_os_abstraction_bm.o 

C_DEPS += \
./component/osa/fsl_os_abstraction_bm.d 


# Each subdirectory must supply rules for building sources it contributes
component/osa/%.o: ../component/osa/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MK64FN1M0VLL12 -DCPU_MK64FN1M0VLL12_cm4 -D_DEBUG=1 -DUSB_STACK_BM -DSDK_I2C_BASED_COMPONENT_USED=1 -DPRINTF_FLOAT_ENABLE=1 -DPRINTF_ADVANCED_ENABLE=1 -DFRDM_K64F -DFREEDOM -DFSL_OSA_BM_TASK_ENABLE=0 -DFSL_OSA_BM_TIMER_CONFIG=0 -DSERIAL_PORT_TYPE_UART=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=1 -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\board" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\source" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\usb\device\source\khci" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\usb\include" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\usb\device\include" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\usb\device\source" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\component\osa" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\drivers" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\device" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\CMSIS" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\component\lists" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\utilities" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\component\serial_manager" -I"C:\Users\yiqi_\Documents\MCUXpressoIDE_11.1.1_3241\workspace\frdmk64f_dev_hid_mouse_lite_bm\component\uart" -O0 -fno-common -g3 -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


