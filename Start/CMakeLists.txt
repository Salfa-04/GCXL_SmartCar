cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
set(CMAKE_PROJECT_NAME              start)
##########################################
### Export CMakeLists.txt for Start Task
##########################################
project(${CMAKE_PROJECT_NAME})
add_library(${CMAKE_PROJECT_NAME} INTERFACE)

file(GLOB STARTS ./*.c)
target_include_directories(${CMAKE_PROJECT_NAME}
    INTERFACE ./Include
)

target_sources(${CMAKE_PROJECT_NAME}
    INTERFACE ${STARTS}
)
