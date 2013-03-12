################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ros/src/seneka_windsensor.cpp 

OBJS += \
./ros/src/seneka_windsensor.o 

CPP_DEPS += \
./ros/src/seneka_windsensor.d 


# Each subdirectory must supply rules for building sources it contributes
ros/src/%.o: ../ros/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/opt/ros/fuerte/include -I/opt/ros/fuerte -I/home/ciby/ros_workspace/sande/seneka/seneka_windsensor/common -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


