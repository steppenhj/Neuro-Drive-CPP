cmake_minimum_required(VERSION 3.10)

project(NeuroDrive)

set(CMAKE_CXX_STANDARD 17)

#header 파일 위치
include_directories(include)

set(SOURCES src/main.cpp src/motor_driver.cpp)

add_executable(neuro_drive ${SOURCES})