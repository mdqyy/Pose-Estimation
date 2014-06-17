################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/3rdParty/bench/spbench/sp_solver.cpp \
../src/3rdParty/bench/spbench/spbenchsolver.cpp \
../src/3rdParty/bench/spbench/test_sparseLU.cpp 

OBJS += \
./src/3rdParty/bench/spbench/sp_solver.o \
./src/3rdParty/bench/spbench/spbenchsolver.o \
./src/3rdParty/bench/spbench/test_sparseLU.o 

CPP_DEPS += \
./src/3rdParty/bench/spbench/sp_solver.d \
./src/3rdParty/bench/spbench/spbenchsolver.d \
./src/3rdParty/bench/spbench/test_sparseLU.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/bench/spbench/%.o: ../src/3rdParty/bench/spbench/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


