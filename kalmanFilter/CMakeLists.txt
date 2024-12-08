CMAKE_MINIMUM_REQUIRED(VERSION 2.8)  
PROJECT(kalman_project) 
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++17 -g")
INCLUDE_DIRECTORIES("/usr/include/eigen3")

include_directories("kalman/")
include_directories("estimateLaneParam/")


file(GLOB Kalman_src "kalman/*.cpp")
file(GLOB estimate_src "estimateLaneParam/*.cpp")

add_executable(kalman main.cpp ${Kalman_src} ${estimate_src})