################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/3rdParty/bench/btl/libs/blitz/btl_blitz.cpp \
../src/3rdParty/bench/btl/libs/blitz/btl_tiny_blitz.cpp 

OBJS += \
./src/3rdParty/bench/btl/libs/blitz/btl_blitz.o \
./src/3rdParty/bench/btl/libs/blitz/btl_tiny_blitz.o 

CPP_DEPS += \
./src/3rdParty/bench/btl/libs/blitz/btl_blitz.d \
./src/3rdParty/bench/btl/libs/blitz/btl_tiny_blitz.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/bench/btl/libs/blitz/%.o: ../src/3rdParty/bench/btl/libs/blitz/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


