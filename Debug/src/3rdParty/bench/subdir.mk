################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/3rdParty/bench/basicbenchmark.cpp \
../src/3rdParty/bench/benchBlasGemm.cpp \
../src/3rdParty/bench/benchCholesky.cpp \
../src/3rdParty/bench/benchEigenSolver.cpp \
../src/3rdParty/bench/benchFFT.cpp \
../src/3rdParty/bench/benchGeometry.cpp \
../src/3rdParty/bench/benchVecAdd.cpp \
../src/3rdParty/bench/bench_gemm.cpp \
../src/3rdParty/bench/bench_norm.cpp \
../src/3rdParty/bench/bench_reverse.cpp \
../src/3rdParty/bench/bench_sum.cpp \
../src/3rdParty/bench/benchmark.cpp \
../src/3rdParty/bench/benchmarkSlice.cpp \
../src/3rdParty/bench/benchmarkX.cpp \
../src/3rdParty/bench/benchmarkXcwise.cpp \
../src/3rdParty/bench/check_cache_queries.cpp \
../src/3rdParty/bench/eig33.cpp \
../src/3rdParty/bench/geometry.cpp \
../src/3rdParty/bench/product_threshold.cpp \
../src/3rdParty/bench/quat_slerp.cpp \
../src/3rdParty/bench/quatmul.cpp \
../src/3rdParty/bench/sparse_cholesky.cpp \
../src/3rdParty/bench/sparse_dense_product.cpp \
../src/3rdParty/bench/sparse_lu.cpp \
../src/3rdParty/bench/sparse_product.cpp \
../src/3rdParty/bench/sparse_randomsetter.cpp \
../src/3rdParty/bench/sparse_setter.cpp \
../src/3rdParty/bench/sparse_transpose.cpp \
../src/3rdParty/bench/sparse_trisolver.cpp \
../src/3rdParty/bench/spmv.cpp \
../src/3rdParty/bench/vdw_new.cpp 

OBJS += \
./src/3rdParty/bench/basicbenchmark.o \
./src/3rdParty/bench/benchBlasGemm.o \
./src/3rdParty/bench/benchCholesky.o \
./src/3rdParty/bench/benchEigenSolver.o \
./src/3rdParty/bench/benchFFT.o \
./src/3rdParty/bench/benchGeometry.o \
./src/3rdParty/bench/benchVecAdd.o \
./src/3rdParty/bench/bench_gemm.o \
./src/3rdParty/bench/bench_norm.o \
./src/3rdParty/bench/bench_reverse.o \
./src/3rdParty/bench/bench_sum.o \
./src/3rdParty/bench/benchmark.o \
./src/3rdParty/bench/benchmarkSlice.o \
./src/3rdParty/bench/benchmarkX.o \
./src/3rdParty/bench/benchmarkXcwise.o \
./src/3rdParty/bench/check_cache_queries.o \
./src/3rdParty/bench/eig33.o \
./src/3rdParty/bench/geometry.o \
./src/3rdParty/bench/product_threshold.o \
./src/3rdParty/bench/quat_slerp.o \
./src/3rdParty/bench/quatmul.o \
./src/3rdParty/bench/sparse_cholesky.o \
./src/3rdParty/bench/sparse_dense_product.o \
./src/3rdParty/bench/sparse_lu.o \
./src/3rdParty/bench/sparse_product.o \
./src/3rdParty/bench/sparse_randomsetter.o \
./src/3rdParty/bench/sparse_setter.o \
./src/3rdParty/bench/sparse_transpose.o \
./src/3rdParty/bench/sparse_trisolver.o \
./src/3rdParty/bench/spmv.o \
./src/3rdParty/bench/vdw_new.o 

CPP_DEPS += \
./src/3rdParty/bench/basicbenchmark.d \
./src/3rdParty/bench/benchBlasGemm.d \
./src/3rdParty/bench/benchCholesky.d \
./src/3rdParty/bench/benchEigenSolver.d \
./src/3rdParty/bench/benchFFT.d \
./src/3rdParty/bench/benchGeometry.d \
./src/3rdParty/bench/benchVecAdd.d \
./src/3rdParty/bench/bench_gemm.d \
./src/3rdParty/bench/bench_norm.d \
./src/3rdParty/bench/bench_reverse.d \
./src/3rdParty/bench/bench_sum.d \
./src/3rdParty/bench/benchmark.d \
./src/3rdParty/bench/benchmarkSlice.d \
./src/3rdParty/bench/benchmarkX.d \
./src/3rdParty/bench/benchmarkXcwise.d \
./src/3rdParty/bench/check_cache_queries.d \
./src/3rdParty/bench/eig33.d \
./src/3rdParty/bench/geometry.d \
./src/3rdParty/bench/product_threshold.d \
./src/3rdParty/bench/quat_slerp.d \
./src/3rdParty/bench/quatmul.d \
./src/3rdParty/bench/sparse_cholesky.d \
./src/3rdParty/bench/sparse_dense_product.d \
./src/3rdParty/bench/sparse_lu.d \
./src/3rdParty/bench/sparse_product.d \
./src/3rdParty/bench/sparse_randomsetter.d \
./src/3rdParty/bench/sparse_setter.d \
./src/3rdParty/bench/sparse_transpose.d \
./src/3rdParty/bench/sparse_trisolver.d \
./src/3rdParty/bench/spmv.d \
./src/3rdParty/bench/vdw_new.d 


# Each subdirectory must supply rules for building sources it contributes
src/3rdParty/bench/%.o: ../src/3rdParty/bench/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -DNDEBUG -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


