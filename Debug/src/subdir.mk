################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Camera.cpp \
../src/Image.cpp \
../src/Main.cpp \
../src/Object.cpp \
../src/Pose.cpp \
../src/Tests.cpp \
../src/solveDLT.cpp \
../src/solveEPNP.cpp \
../src/solveLM.cpp \
../src/solveOI.cpp \
../src/solvePosit.cpp 

OBJS += \
./src/Camera.o \
./src/Image.o \
./src/Main.o \
./src/Object.o \
./src/Pose.o \
./src/Tests.o \
./src/solveDLT.o \
./src/solveEPNP.o \
./src/solveLM.o \
./src/solveOI.o \
./src/solvePosit.o 

CPP_DEPS += \
./src/Camera.d \
./src/Image.d \
./src/Main.d \
./src/Object.d \
./src/Pose.d \
./src/Tests.d \
./src/solveDLT.d \
./src/solveEPNP.d \
./src/solveLM.d \
./src/solveOI.d \
./src/solvePosit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


