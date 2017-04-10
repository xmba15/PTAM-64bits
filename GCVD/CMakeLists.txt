cmake_minimum_required(VERSION 2.8)

project(CameraCalibrator CXX)

set(CMAKE_C_FLAGS_RELEASE "-Wall -Wextra -O2")

set(CMAKE_C_FLAGS_DEBUG "-Wall -Wextra -g")

file(GLOB gcvdSrcs ${CMAKE_SOURCE_DIR}/GCVD/*.h*
                   ${CMAKE_SOURCE_DIR}/GCVD/*.cpp)

add_library(GCVD STATIC ${gcvdSrcs})
