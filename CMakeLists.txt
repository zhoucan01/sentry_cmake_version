cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME infantry_25)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ==================== application 层 ====================
    application/CAN_receive.c
    application/CAN_transmit.c
    application/ins_task.c
    application/remote_control.c

    # ==================== branch 层 ====================
    branch/chassis_control.c
    branch/decision.c
    branch/gimbal_big_yaw.c
    branch/motor.c
    branch/navigation.c
    branch/small_gimbal.c
    branch/system.c

    # ==================== bsp/boards 层 ====================
    bsp/boards/bsp_can.c
    bsp/boards/bsp_delay.c
    bsp/boards/bsp_dwt.c
    bsp/boards/bsp_led.c
    bsp/boards/bsp_pid.c
    bsp/boards/bsp_PWM.c
    bsp/boards/bsp_rc.c
    bsp/boards/bsp_sin_cos_atan_sqrt.c
    bsp/boards/bsp_transmit.c
    bsp/boards/kalman.c

    # ==================== Components 层 ====================
    Components/kalman_filter.c
    Components/user_lib.c
    Components/Algorithm/QuaternionEKF.c
    Components/Controller/controller.c
    Components/Devices/BMI088driver.c
    Components/Devices/BMI088Middleware.c
    Components/Devices/transfer_function.c

    # ==================== essemi 调试打印 ====================
    essemi/essemi_swd_print.c
    essemi/essemi_swd_print_printf.c

    # ==================== Nautilus 模块 ====================
    Nautilus_UI/Nautilus_UI.c
    Nautilus_UI/Sentry_cmd.c
    Nautilus_Vision/Nautilus_Vision.c

    # ==================== referee 裁判系统 ====================
    referee/bsp_usart.c
    referee/CRC8_CRC16.c
    referee/fifo.c
    referee/referee_usart_task.c
    referee/referee.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # ==================== 用户头文件路径 ====================
    application
    branch
    bsp/boards
    Components
    Components/Algorithm
    Components/Algorithm/Include
    Components/Controller
    Components/Devices
    essemi
    Nautilus_UI
    Nautilus_Vision
    referee
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # FPU 相关 (对应 Keil 中的 __FPU_USED=1U, __FPU_PRESENT=1U)
    __FPU_USED=1U
    __FPU_PRESENT=1U

    # CMSIS-DSP 宏定义 (Cortex-M4)
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ==================== CMSIS-DSP 库 ====================
set(CMSIS_DSP_Sources
    # 公共表（三角函数/矩阵等查表数据）
    Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c

    # 快速数学函数
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c

    # 矩阵运算
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
)

add_library(CMSIS_DSP OBJECT ${CMSIS_DSP_Sources})
target_include_directories(CMSIS_DSP PUBLIC Drivers/CMSIS/DSP/Include)
target_link_libraries(CMSIS_DSP PUBLIC stm32cubemx)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSIS_DSP

    # Add user defined libraries
)

# ==================== 烧录目标 (ST-Link) ====================
# 用法: cmake --build build/Debug --target flash
add_custom_target(hex DEPENDS ${CMAKE_PROJECT_NAME}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex"
)

add_custom_target(flash DEPENDS hex
    COMMAND "C:/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe" -c SWD UR -P ${CMAKE_PROJECT_NAME}.hex -Rst -Run
    COMMENT "Flashing ${CMAKE_PROJECT_NAME}.hex via ST-Link"
)
