#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME ERG-Project)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.17)

#引用ARMClang_Cortex-M4.cmake
include(ARMClang_Cortex-M4.cmake)

CMSIS_EnableFPU(TRUE)

#设置Sections脚本路径
CMSIS_SetSectionsScriptPath(${CMAKE_HOME_DIRECTORY}/STM32G431VB.sct)

CMSIS_CrossCompilingConfiguration()

# project settings
project(ERG26_G4 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_definitions(SYSVIEW_DEBUG)
    add_compile_options(-Og -g -gdwarf-2)
endif ()

include_directories(Board/ERG26/Inc
                    ThirdParty/STM32G4xx_HAL_Driver/Inc
                    ThirdParty/STM32G4xx_HAL_Driver/Inc/Legacy
                    ThirdParty/CMSIS/DSP/Include
                    ThirdParty/CMSIS/Device/ST/STM32G4xx/Include
                    ThirdParty/CMSIS/Include)

add_definitions(-DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32G431xx -DFPU_FPV4)

file(GLOB_RECURSE SOURCES 
    "Board/ERG26/*.*" 
    "ThirdParty/*.*"
    "${STARTUP_FILE}")

CMSIS_CompileELF()

CMSIS_CompileHex()
