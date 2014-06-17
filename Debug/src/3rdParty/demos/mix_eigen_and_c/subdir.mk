################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/3rdParty/demos/mix_eigen_and_c/binary_library.cpp 

C_SRCS += \
../src/3rdParty/demos/mix_eigen_and_c/example.c 

OBJS += \
./src/3rdParty/demos/mix_eigen_and_c/binary_library.o \
./src/3rdParty/demos/mix_eigen_and_c/example.o 

C_DEPS += \
./src/3rdParty/demos/mix_eigen_and_c/example.d 

CPP_DEPS += \
./src/3rdParty/demos/mix_eigen_and_c/binary_library.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/demos/mix_eigen_and_c/%.o: ../src/3rdParty/demos/mix_eigen_and_c/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/3rdParty/demos/mix_eigen_and_c/%.o: ../src/3rdParty/demos/mix_eigen_and_c/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


