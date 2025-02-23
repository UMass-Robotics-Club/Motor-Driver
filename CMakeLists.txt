cmake_minimum_required(VERSION 3.13)

project(motor_driver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


file(GLOB cSources ./src/*.c)
add_executable(${PROJECT_NAME} ${cSources})

target_include_directories(${PROJECT_NAME} PUBLIC ./include)