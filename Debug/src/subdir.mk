################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/config.c \
../src/iors_main.c \
../src/radio.c \
../src/str_util.c 

C_DEPS += \
./src/config.d \
./src/iors_main.d \
./src/radio.d \
./src/str_util.d 

OBJS += \
./src/config.o \
./src/iors_main.o \
./src/radio.o \
./src/str_util.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I../inc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/config.d ./src/config.o ./src/iors_main.d ./src/iors_main.o ./src/radio.d ./src/radio.o ./src/str_util.d ./src/str_util.o

.PHONY: clean-src

