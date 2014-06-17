################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/3rdParty/bench/btl/libs/eigen3/btl_tiny_eigen3.cpp \
../src/3rdParty/bench/btl/libs/eigen3/main_adv.cpp \
../src/3rdParty/bench/btl/libs/eigen3/main_linear.cpp \
../src/3rdParty/bench/btl/libs/eigen3/main_matmat.cpp \
../src/3rdParty/bench/btl/libs/eigen3/main_vecmat.cpp 

OBJS += \
./src/3rdParty/bench/btl/libs/eigen3/btl_tiny_eigen3.o \
./src/3rdParty/bench/btl/libs/eigen3/main_adv.o \
./src/3rdParty/bench/btl/libs/eigen3/main_linear.o \
./src/3rdParty/bench/btl/libs/eigen3/main_matmat.o \
./src/3rdParty/bench/btl/libs/eigen3/main_vecmat.o 

CPP_DEPS += \
./src/3rdParty/bench/btl/libs/eigen3/btl_tiny_eigen3.d \
./src/3rdParty/bench/btl/libs/eigen3/main_adv.d \
./src/3rdParty/bench/btl/libs/eigen3/main_linear.d \
./src/3rdParty/bench/btl/libs/eigen3/main_matmat.d \
./src/3rdParty/bench/btl/libs/eigen3/main_vecmat.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/bench/btl/libs/eigen3/%.o: ../src/3rdParty/bench/btl/libs/eigen3/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


