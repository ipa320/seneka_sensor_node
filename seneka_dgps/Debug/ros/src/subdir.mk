################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ros/src/seneka_dgps.cpp 

OBJS += \
./ros/src/seneka_dgps.o 

CPP_DEPS += \
./ros/src/seneka_dgps.d 


# Each subdirectory must supply rules for building sources it contributes
ros/src/%.o: ../ros/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


