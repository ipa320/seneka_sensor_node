################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../common/src/SerialIO.cpp \
../common/src/windsensor.cpp 

OBJS += \
./common/src/SerialIO.o \
./common/src/windsensor.o 

CPP_DEPS += \
./common/src/SerialIO.d \
./common/src/windsensor.d 


# Each subdirectory must supply rules for building sources it contributes
common/src/%.o: ../common/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/opt/ros/fuerte/include -I/opt/ros/fuerte -I/home/ciby/ros_workspace/sande/seneka/seneka_windsensor/common -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


