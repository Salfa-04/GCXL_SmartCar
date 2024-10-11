cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
set(CMAKE_PROJECT_NAME              applications)
##########################################
### Export CMakeLists.txt for Applications
##########################################
project(${CMAKE_PROJECT_NAME})
add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_link_libraries(${CMAKE_PROJECT_NAME} INTERFACE
    start carmove
)



##########################################
### Start Task
##########################################

add_library(start INTERFACE)
file(GLOB APPS_START Start/*.c)
target_include_directories(start INTERFACE
    Start/Include
)

target_sources(start INTERFACE
    ${APPS_START}
)

##########################################
### Blinky Task
##########################################

add_library(carmove INTERFACE)
file(GLOB APPS_CARMOVE CarMove/*.c)
target_include_directories(carmove INTERFACE
    CarMove
)

target_sources(carmove INTERFACE
    ${APPS_CARMOVE}
)
