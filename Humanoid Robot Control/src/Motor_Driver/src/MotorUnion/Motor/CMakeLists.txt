cmake_minimum_required(VERSION 3.5)
project(Motor)

add_subdirectory(dynamixel)
add_subdirectory(motor)

add_library(Motor SHARED Motor.cpp MotorXm.cpp MotorPro.cpp MotorProPlus.cpp MotorMx.cpp )
target_link_libraries(Motor DynamixelSdk motor)
