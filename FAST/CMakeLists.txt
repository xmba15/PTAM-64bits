cmake_minimum_required(VERSION 2.8)

project(CameraCalibrator C)

set(CMAKE_C_FLAGS_RELEASE "-Wall -Wextra -O2")

set(CMAKE_C_FLAGS_DEBUG "-Wall -Wextra -g")

file(GLOB fastSrcs ${CMAKE_SOURCE_DIR}/FAST/*.h*
                   ${CMAKE_SOURCE_DIR}/FAST/*.cpp)

add_library(FAST STATIC ${fastSrcs})

