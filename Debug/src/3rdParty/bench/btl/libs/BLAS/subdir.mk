################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/3rdParty/bench/btl/libs/BLAS/main.cpp 

OBJS += \
./src/3rdParty/bench/btl/libs/BLAS/main.o 

CPP_DEPS += \
./src/3rdParty/bench/btl/libs/BLAS/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/bench/btl/libs/BLAS/%.o: ../src/3rdParty/bench/btl/libs/BLAS/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


