################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/uart/fsl_adapter_uart.c 

OBJS += \
./component/uart/fsl_adapter_uart.o 

C_DEPS += \
./component/uart/fsl_adapter_uart.d 


# Each subdirectory must supply rules for building sources it contributes
component/uart/%.o: ../component/uart/%.c component/uart/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MK66FN2M0VMD18 -DCPU_MK66FN2M0VMD18_cm4 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K66F -DFREEDOM -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\board" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\source" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\utilities" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\drivers" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\device" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\component\uart" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\component\lists" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_problem2\CMSIS" -O0 -fno-common -g3 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


