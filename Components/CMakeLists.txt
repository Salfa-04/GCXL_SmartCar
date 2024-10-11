cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
set(CMAKE_PROJECT_NAME              components)
##########################################
### Export CMakeLists.txt for Components
##########################################
project(${CMAKE_PROJECT_NAME})
add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_link_libraries(${CMAKE_PROJECT_NAME} INTERFACE
    fset imu mot cha
)



##########################################
### fSetting Driver
##########################################

add_library(fset INTERFACE)
file(GLOB COMPS_EXAMPLE fSetting/*.c)
target_include_directories(fset INTERFACE
    fSetting
)
target_sources(fset INTERFACE
    ${COMPS_EXAMPLE}
)

##########################################
### HWT101 Driver
##########################################

add_library(imu INTERFACE)
file(GLOB COMPS_EXAMPLE HWT101/*.c)
target_include_directories(imu INTERFACE
    HWT101
)
target_sources(imu INTERFACE
    ${COMPS_EXAMPLE}
)

##########################################
### Motor Driver
##########################################

add_library(mot INTERFACE)
file(GLOB COMPS_EXAMPLE Motor/*.c)
target_include_directories(mot INTERFACE
    Motor
)
target_sources(mot INTERFACE
    ${COMPS_EXAMPLE}
)

##########################################
### Chassis Control Driver
##########################################

add_library(cha INTERFACE)
file(GLOB COMPS_EXAMPLE Chassis/*.c)
target_include_directories(cha INTERFACE
    Chassis
)
target_sources(cha INTERFACE
    ${COMPS_EXAMPLE}
)

##########################################
### Example Driver
##########################################

add_library(example INTERFACE)
file(GLOB COMPS_EXAMPLE Example/*.c)
target_compile_definitions(example INTERFACE
)

target_include_directories(example INTERFACE
    Example
)

target_sources(example INTERFACE
    ${COMPS_EXAMPLE}
)
