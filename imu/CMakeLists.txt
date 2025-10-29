cmake_minimum_required(VERSION 3.10.0)
project(imu VERSION 0.1.0 LANGUAGES C CXX)

add_executable(imu main.cpp madgwick_filter.cpp)

