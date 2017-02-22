cmake_minimum_required(VERSION 2.8)

project(CameraCalibrator C)

set(CMAKE_BUILD_TYPE RELEASE)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#pthread
set(pthreadSrc "C:/workSpace/environment64/pthreads")
# include_directories("C:/workSpace/environment64/pthreads/include")
include_directories(${pthreadSrc}/include)
link_directories(${pthreadSrc}/x64/lib)
file(GLOB_RECURSE 3rdparty ${3rdparty} ${pthreadSrc}/x64/lib/*.lib)

#glew
set(glewSrc "C:/workSpace/environment64/glew-2.0.0")
include_directories(${glewSrc}/include)
link_directories(${glewSrc}/lib)
file(GLOB_RECURSE 3rdparty ${3rdparty} ${glewSrc}/lib/*.lib)

#blas, lapack
set(lapackSrc "C:/workSpace/environment64/lapack")
include_directories(${lapackSrc}/include)
link_directories(${lapackSrc}/lib)
file(GLOB_RECURSE 3rdparty ${3rdparty} ${lapackSrc}/lib/*.lib)

set(3rdparty ${3rdparty} opengl32.lib)

message(${3rdparty})

# source code
set(camCalib
   OpenGL.h
   OpenCV.h
   GLWindowMenu.h
   GLWindow2.h
   additionalUtility.h
   VideoSourceOpenCV.h  
   ATANCamera.h
   CalibCornerPatch.h
   CalibImage.h
   CameraCalibrator.h

   additionalUtility.cpp
   GLWindowMenu.cpp
   GLWindow2.cpp
   additionalUtility.cpp
   VideoSourceOpenCV.cpp
   ATANCamera.cpp
   CalibCornerPatch.cpp
   CalibImage.cpp
   CameraCalibrator.cpp
)

add_subdirectory(FAST)
add_subdirectory(GCVD)
add_subdirectory(Persistence)

# message(${CMAKE_BINARY_DIR})
add_executable(${PROJECT_NAME} ${camCalib})
target_link_libraries(CameraCalibrator ${OpenCV_LIBS} ${3rdparty} FAST GCVD Persistence)
