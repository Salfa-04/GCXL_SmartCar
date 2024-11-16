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
file(GLOB COMPS_FSET fSetting/*.c)
target_include_directories(fset INTERFACE
    fSetting
)
target_sources(fset INTERFACE
    ${COMPS_FSET}
)

##########################################
### HWT101 Driver
##########################################

add_library(imu INTERFACE)
file(GLOB COMPS_IMU HWT101/*.c)
target_include_directories(imu INTERFACE
    HWT101
)
target_sources(imu INTERFACE
    ${COMPS_IMU}
)

##########################################
### Motor Driver
##########################################

add_library(mot INTERFACE)
file(GLOB COMPS_MOT Motor/*.c)
target_include_directories(mot INTERFACE
    Motor
)
target_sources(mot INTERFACE
    ${COMPS_MOT}
)

##########################################
### Chassis Control Driver
##########################################

add_library(cha INTERFACE)
file(GLOB COMPS_CHA Chassis/*.c)
target_include_directories(cha INTERFACE
    Chassis
)
target_sources(cha INTERFACE
    ${COMPS_CHA}
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
