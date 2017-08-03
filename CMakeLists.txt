# @file   CMakeLists.txt
# @Author Benjamin Bercovici (bebe0705@colorado.edu)
# @date   July, 2017
# @brief  CMake listing enabling compilation and installation of the RBK library

 
cmake_minimum_required(VERSION 2.8)

get_filename_component(dirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(LIB_NAME ${dirName} CACHE STRING "Name of library to be created.")

project(${LIB_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/source/cmake)

add_definitions(-Wall -O2 )
message(${CMAKE_MODULE_PATH})
find_package(Armadillo REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${LIB_NAME} SHARED source/RigidBodyKinematics.cpp)
target_link_libraries(${LIB_NAME} ${ARMADILLO_LIBRARIES})

install (TARGETS ${LIB_NAME} DESTINATION /usr/local/lib)
install (FILES ${PROJECT_SOURCE_DIR}/include/RigidBodyKinematics.hpp DESTINATION /usr/local/include)
install (FILES ${PROJECT_SOURCE_DIR}/RigidBodyKinematicsConfig.cmake DESTINATION /usr/local/lib/cmake/RigidBodyKinematics/)
