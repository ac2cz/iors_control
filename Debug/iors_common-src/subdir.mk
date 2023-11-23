################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/g0kla/Desktop/workspace/iors_common/src/str_util.c 

C_DEPS += \
./iors_common-src/str_util.d 

OBJS += \
./iors_common-src/str_util.o 


# Each subdirectory must supply rules for building sources it contributes
iors_common-src/str_util.o: /home/g0kla/Desktop/workspace/iors_common/src/str_util.c iors_common-src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I../inc -I../../iors_common/inc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-iors_common-2d-src

clean-iors_common-2d-src:
	-$(RM) ./iors_common-src/str_util.d ./iors_common-src/str_util.o

.PHONY: clean-iors_common-2d-src

