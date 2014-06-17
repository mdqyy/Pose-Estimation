################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../src/3rdParty/bench/btl/data/mean.cxx \
../src/3rdParty/bench/btl/data/regularize.cxx \
../src/3rdParty/bench/btl/data/smooth.cxx 

OBJS += \
./src/3rdParty/bench/btl/data/mean.o \
./src/3rdParty/bench/btl/data/regularize.o \
./src/3rdParty/bench/btl/data/smooth.o 

CXX_DEPS += \
./src/3rdParty/bench/btl/data/mean.d \
./src/3rdParty/bench/btl/data/regularize.d \
./src/3rdParty/bench/btl/data/smooth.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/bench/btl/data/%.o: ../src/3rdParty/bench/btl/data/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


