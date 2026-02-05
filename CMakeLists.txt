cmake_minimum_required(VERSION 3.10)
project(NeuroDrive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(neuro_drive control_core_threaded.cpp)