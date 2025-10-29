cmake_minimum_required(VERSION 3.10.0)
project(lidar VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RAYLIB_DIR "${CMAKE_SOURCE_DIR}/raylib")
set(RAYLIB_INCLUDE_DIR "${RAYLIB_DIR}/include")
set(RAYLIB_LIB_DIR "${RAYLIB_DIR}/lib")


add_executable(lidar main.cpp ld19_gatherer.cpp ld19_gatherer.hpp)

target_include_directories(lidar PRIVATE ${RAYLIB_INCLUDE_DIR})

target_link_libraries(lidar
    PRIVATE
        ${RAYLIB_LIB_DIR}/libraylib.a
        m dl pthread GL X11
)

