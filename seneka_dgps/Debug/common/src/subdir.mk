################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../common/src/Dgps.cpp \
../common/src/SerialIO.cpp 

OBJS += \
./common/src/Dgps.o \
./common/src/SerialIO.o 

CPP_DEPS += \
./common/src/Dgps.d \
./common/src/SerialIO.d 


# Each subdirectory must supply rules for building sources it contributes
common/src/%.o: ../common/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


