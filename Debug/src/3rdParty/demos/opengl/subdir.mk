################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/3rdParty/demos/opengl/camera.cpp \
../src/3rdParty/demos/opengl/gpuhelper.cpp \
../src/3rdParty/demos/opengl/icosphere.cpp \
../src/3rdParty/demos/opengl/quaternion_demo.cpp \
../src/3rdParty/demos/opengl/trackball.cpp 

OBJS += \
./src/3rdParty/demos/opengl/camera.o \
./src/3rdParty/demos/opengl/gpuhelper.o \
./src/3rdParty/demos/opengl/icosphere.o \
./src/3rdParty/demos/opengl/quaternion_demo.o \
./src/3rdParty/demos/opengl/trackball.o 

CPP_DEPS += \
./src/3rdParty/demos/opengl/camera.d \
./src/3rdParty/demos/opengl/gpuhelper.d \
./src/3rdParty/demos/opengl/icosphere.d \
./src/3rdParty/demos/opengl/quaternion_demo.d \
./src/3rdParty/demos/opengl/trackball.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/demos/opengl/%.o: ../src/3rdParty/demos/opengl/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


