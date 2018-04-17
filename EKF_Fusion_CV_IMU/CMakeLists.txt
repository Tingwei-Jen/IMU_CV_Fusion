cmake_minimum_required( VERSION 2.8 )
project ( Extended_Kalman_Filter )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
############### source and test ######################
set(SOURCE_FILES kalman_filter.cpp fusion.cpp readwritedata.cpp hl_pass_filter.cpp main.cpp)
add_executable(${PROJECT_NAME}  ${SOURCE_FILES})

