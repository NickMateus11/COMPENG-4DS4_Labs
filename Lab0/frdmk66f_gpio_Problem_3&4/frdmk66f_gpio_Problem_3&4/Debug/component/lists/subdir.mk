################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/lists/fsl_component_generic_list.c 

OBJS += \
./component/lists/fsl_component_generic_list.o 

C_DEPS += \
./component/lists/fsl_component_generic_list.d 


# Each subdirectory must supply rules for building sources it contributes
component/lists/%.o: ../component/lists/%.c component/lists/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MK66FN2M0VMD18 -DCPU_MK66FN2M0VMD18_cm4 -DFRDM_K66F -DFREEDOM -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\board" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\source" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\utilities" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\drivers" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\device" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\component\uart" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\component\lists" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\CMSIS" -I"C:\Users\halo3\git\COMPENG-4DS4_Labs\Lab0\frdmk66f_gpio_Problem_3&4\frdmk66f_gpio_Problem_3&4\frdmk66f\driver_examples\gpio\led_output" -O0 -fno-common -g3 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


