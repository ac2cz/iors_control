################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/config.c \
../src/iors_command.c \
../src/iors_main.c \
../src/radio.c 

C_DEPS += \
./src/config.d \
./src/iors_command.d \
./src/iors_main.d \
./src/radio.d 

OBJS += \
./src/config.o \
./src/iors_command.o \
./src/iors_main.o \
./src/radio.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I../inc -I/usr/local/include/iors_common -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/config.d ./src/config.o ./src/iors_command.d ./src/iors_command.o ./src/iors_main.d ./src/iors_main.o ./src/radio.d ./src/radio.o

.PHONY: clean-src

