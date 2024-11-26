cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
set(CMAKE_PROJECT_NAME              drivers)
##########################################
### Export CMakeLists.txt for Drivers
##########################################
project(${CMAKE_PROJECT_NAME})
add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_link_libraries(${CMAKE_PROJECT_NAME} INTERFACE
    stm32cubemx
)



##########################################
### STM32CubeMX CMSIS&HAL Driver
##########################################

add_library(stm32cubemx INTERFACE)
file(GLOB HAL_SOURCE F4HAL/Source/*.c)
target_compile_definitions(stm32cubemx INTERFACE
    $<$<CONFIG:Debug>:DEBUG>
    USE_HAL_DRIVER STM32F407xx
)

target_include_directories(stm32cubemx INTERFACE
    CMSIS/Device/Include CMSIS/Include
    F4HAL/Include F4HAL/Include/Legacy
)

target_sources(stm32cubemx INTERFACE ${HAL_SOURCE})
