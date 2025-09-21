cmake_minimum_required(VERSION 3.16)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ① 加载工具链（所有交叉编译器 / MCU_FLAGS / 链接脚本 都在里面）
include(${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

project(RM_hero_ctype-241124_-main C ASM)          # 工程名，想改就改
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ② 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# ③ 宏定义（换芯片改这里）
add_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32F407xx      # F1 就写 STM32F103xB
    -DARM_MATH_CM4
)

# ④ 递归包含头文件
function(include_sub_directories_recursively root_dir)
    if(IS_DIRECTORY ${root_dir})
        include_directories(${root_dir})
    endif()
    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)
    foreach(sub ${ALL_SUB})
        if(IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub})
        endif()
    endforeach()
endfunction()
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Drivers)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Middlewares)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Inc)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Src)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/modules)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/application)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/bsp)
# ⑤ 源文件
file(GLOB_RECURSE SOURCES
     "Drivers/*.c"
     "Middlewares/*.c"
     "Inc/*.c"
     "Src/*.c"
     "modules/*.c"
     "application/*.c"
     "bsp/*.c"   
)
set(ASM_SOURCES
    startup_stm32f407xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
)
set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# ⑥ 优化级别
if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
    add_compile_options(-Ofast)
else()
    add_compile_options(-Og -g -gdwarf-2)
    add_definitions(-DESC_DEBUG)
endif()

# ⑦ 生成 elf
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libCMSISDSP.a  # 显式绝对路径
    m                                                             # libm
    nosys                                                         # 桩
)

# ⑧ 生成 hex/bin + 打印大小
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex  $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>   # ← 用生成器表达式
)
# ⑨ 一键 J-Flash 下载（原来就有的，保留）
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing ${PROJECT_NAME}.hex ..."
    COMMAND JFlash -openprj ${CMAKE_SOURCE_DIR}/stm32.jflash
                   -open ${PROJECT_NAME}.hex -8000000 -auto -startapp -exit
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing complete!"
)