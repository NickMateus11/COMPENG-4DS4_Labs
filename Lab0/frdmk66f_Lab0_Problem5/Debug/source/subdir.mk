################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/hello_world.c \
../source/semihost_hardfault.c 

OBJS += \
./source/hello_world.o \
./source/semihost_hardfault.o 

C_DEPS += \
./source/hello_world.d \
./source/semihost_hardfault.d 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_MK66FN2M0VMD18 -DCPU_MK66FN2M0VMD18_cm4 -DPRINTF_FLOAT_ENABLE=0 -DSCANF_FLOAT_ENABLE=0 -DPRINTF_ADVANCED_ENABLE=0 -DSCANF_ADVANCED_ENABLE=0 -DFRDM_K66F -DFREEDOM -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -DSDK_OS_BAREMETAL -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\utilities" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\drivers" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\component\uart" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\component\lists" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\CMSIS" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\device" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\board" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\source" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\utilities" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\drivers" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\device" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\component\uart" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\component\lists" -I"C:\Users\mateu\AppData\Roaming\SPB_Data\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_Lab0_Problem5\CMSIS" -O0 -fno-common -g3 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


