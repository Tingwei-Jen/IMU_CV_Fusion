cmake_minimum_required( VERSION 2.8 )
project ( imu_high_and_low_pass_filter )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

############### source and test ######################
set(SOURCE_FILES main.cpp)
add_executable(${PROJECT_NAME}  ${SOURCE_FILES})

