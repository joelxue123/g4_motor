#设置CMake最低支持版本
cmake_minimum_required(VERSION 3.17)

#交叉编译：设置目标机器类型
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR Cortex-M4)

#设置编译器
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

#接口：用户变量
set(FPU FALSE CACHE STRING "global") #设置默认FPU选项

#接口: FPU使能控制
function(CMSIS_EnableFPU isEnabled)
    set(FPU ${isEnabled} CACHE STRING "global")
endfunction()

#接口：设置Sections脚本
function(CMSIS_SetSectionsScriptPath path)
    set(SECTIONS_SCRIPT_PATH ${path} CACHE STRING "global")
endfunction()

#接口：配置交叉编译
function(CMSIS_CrossCompilingConfiguration)

    add_compile_options(-ffast-math -fno-finite-math-only)
    add_compile_options(-Wdouble-promotion -Wfloat-conversion)
    add_compile_options(-mcpu=cortex-m4 -mthumb)
    add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

    #设置FPU,Target参数
    add_compile_definitions(__VFP_FP__)
    add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
    add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

    set(STARTUP_FILE "Startup/Gcc/*.*" CACHE STRING "global")
    #判断链接脚本是否存在
    if (NOT SECTIONS_SCRIPT_PATH)
        message(FATAL_ERROR "You not set SECTIONS_SCRIPT_PATH!")
    endif ()

    #设置链接选项
    add_link_options(-T ${SECTIONS_SCRIPT_PATH})
    #使能汇编
    enable_language(ASM)

endfunction()

#接口: 生成elf文件
function(CMSIS_CompileELF)
    add_link_options(-lc -lm)
    add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
    add_link_options(-ffast-math -fno-finite-math-only -specs=nosys.specs -specs=nano.specs)
    add_link_options(-mcpu=cortex-m4 -mthumb)

    add_executable(${PROJECT_NAME}.elf ${SOURCES})
endfunction()

#接口: 生成hex和bin文件
function(CMSIS_CompileHex)
    set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
    set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
            COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
            COMMENT "Building ${HEX_FILE}
    Building ${BIN_FILE}")
endfunction()
