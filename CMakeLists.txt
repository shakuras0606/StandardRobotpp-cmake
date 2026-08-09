cmake_minimum_required(VERSION 3.20)

project(standard_robot LANGUAGES C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(APPLICATION_DIR "${PROJECT_DIR}/application")
set(BSP_DIR "${PROJECT_DIR}/bsp")
set(COMPONENTS_DIR "${PROJECT_DIR}/components")
set(DRIVERS_DIR "${PROJECT_DIR}/Drivers")
set(MIDDLEWARES_DIR "${PROJECT_DIR}/Middlewares")
set(SRC_DIR "${PROJECT_DIR}/Src")
set(INC_DIR "${PROJECT_DIR}/Inc")

# Cortex-M4F (single-precision FPU), hard-float ABI to match the prebuilt
# ARM Compiler 5 libraries (AHRS.lib, arm_cortexM4lf_math.lib: VFPv4-D16,
# Tag_ABI_VFP_args=VFP registers). Keil target: -O1, warning level 2.
set(DEVICE_FLAGS
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
)

# Compiler-specific startup (GNU asm). Keil uses MDK-ARM/startup_stm32f407xx.s.
set(STARTUP_SOURCES
    ${DRIVERS_DIR}/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
)

set(CUBEMX_SOURCES
    ${SRC_DIR}/main.c
    ${SRC_DIR}/gpio.c
    ${SRC_DIR}/freertos.c
    ${SRC_DIR}/adc.c
    ${SRC_DIR}/can.c
    ${SRC_DIR}/crc.c
    ${SRC_DIR}/dma.c
    ${SRC_DIR}/i2c.c
    ${SRC_DIR}/rng.c
    ${SRC_DIR}/rtc.c
    ${SRC_DIR}/spi.c
    ${SRC_DIR}/tim.c
    ${SRC_DIR}/usart.c
    ${SRC_DIR}/usb_device.c
    ${SRC_DIR}/usbd_conf.c
    ${SRC_DIR}/usbd_desc.c
    ${SRC_DIR}/usbd_cdc_if.c
    ${SRC_DIR}/stm32f4xx_it.c
    ${SRC_DIR}/stm32f4xx_hal_msp.c
    ${SRC_DIR}/system_stm32f4xx.c
)

set(HAL_SOURCES
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# FreeRTOS V10.0.1. Keil uses the RVDS port; GCC uses the GCC/ARM_CM4F port.
set(FREERTOS_SOURCES
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/croutine.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/event_groups.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/list.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/queue.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/tasks.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/timers.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

set(USB_SOURCES
    ${MIDDLEWARES_DIR}/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${MIDDLEWARES_DIR}/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${MIDDLEWARES_DIR}/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${MIDDLEWARES_DIR}/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

set(BSP_SOURCES
    ${BSP_DIR}/boards/bsp_adc.c
    ${BSP_DIR}/boards/bsp_buzzer.c
    ${BSP_DIR}/boards/bsp_can.c
    ${BSP_DIR}/boards/bsp_crc32.c
    ${BSP_DIR}/boards/bsp_delay.c
    ${BSP_DIR}/boards/bsp_flash.c
    ${BSP_DIR}/boards/bsp_fric.c
    ${BSP_DIR}/boards/bsp_imu_pwm.c
    ${BSP_DIR}/boards/bsp_i2c.c
    ${BSP_DIR}/boards/bsp_laser.c
    ${BSP_DIR}/boards/bsp_led.c
    ${BSP_DIR}/boards/bsp_rc.c
    ${BSP_DIR}/boards/bsp_rng.c
    ${BSP_DIR}/boards/bsp_spi.c
    ${BSP_DIR}/boards/bsp_usart.c
    ${BSP_DIR}/boards/bsp_uart.c
    ${BSP_DIR}/boards/bsp_pwm.c
)

set(DEVICE_SOURCES
    ${COMPONENTS_DIR}/devices/OLED.c
    ${COMPONENTS_DIR}/devices/BMI088driver.c
    ${COMPONENTS_DIR}/devices/BMI088Middleware.c
    ${COMPONENTS_DIR}/devices/ist8310driver.c
    ${COMPONENTS_DIR}/devices/ist8310driver_middleware.c
)

set(ALGORITHM_SOURCES
    ${COMPONENTS_DIR}/algorithm/AHRS_middleware.c
    ${COMPONENTS_DIR}/algorithm/user_lib.c
)

# Prebuilt ARM Compiler 5 libraries (ELF archives, hard-float VFPv4-D16 ABI).
# Copied into the build directory at configure time so the link line uses
# ASCII relative paths (GNU ld cannot open non-ASCII paths on this host).
set(ALGORITHM_LIBRARIES
    ${COMPONENTS_DIR}/algorithm/AHRS.lib
    ${COMPONENTS_DIR}/algorithm/arm_cortexM4lf_math.lib
)

file(COPY ${ALGORITHM_LIBRARIES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(SUPPORT_SOURCES
    ${COMPONENTS_DIR}/support/CRC8_CRC16.c
    ${COMPONENTS_DIR}/support/fifo.c
    ${COMPONENTS_DIR}/support/mem_mang4.c
    ${COMPONENTS_DIR}/support/kalman_filter.c
    ${COMPONENTS_DIR}/support/signal_generator.c
    ${COMPONENTS_DIR}/support/clist.c
    ${COMPONENTS_DIR}/controller/pid.c
)

set(APPLICATION_SOURCES
    ${APPLICATION_DIR}/assist/detect_task.c
    ${APPLICATION_DIR}/assist/develop_task.c
    ${APPLICATION_DIR}/assist/data_exchange.c
    ${APPLICATION_DIR}/chassis/chassis.c
    ${APPLICATION_DIR}/chassis/chassis_power_control.c
    ${APPLICATION_DIR}/chassis/chassis_task.c
    ${APPLICATION_DIR}/chassis/chassis_balance.c
    ${APPLICATION_DIR}/chassis/chassis_balance_extras.c
    ${APPLICATION_DIR}/chassis/chassis_mecanum.c
    ${APPLICATION_DIR}/chassis/chassis_omni.c
    ${APPLICATION_DIR}/chassis/chassis_steering.c
    ${APPLICATION_DIR}/communication/communication_task.c
    ${APPLICATION_DIR}/communication/communication.c
    ${APPLICATION_DIR}/communication/usb_task.c
    ${APPLICATION_DIR}/communication/Vofa.c
    ${APPLICATION_DIR}/communication/Vofa_stm32.c
    ${APPLICATION_DIR}/custom_controller/custom_controller.c
    ${APPLICATION_DIR}/custom_controller/custom_controller_task.c
    ${APPLICATION_DIR}/custom_controller/custom_controller_penguin_mini.c
    ${APPLICATION_DIR}/custom_controller/custom_controller_engineer.c
    ${APPLICATION_DIR}/custom_controller/custom_controller_connect.c
    ${APPLICATION_DIR}/gimbal/gimbal.c
    ${APPLICATION_DIR}/gimbal/gimbal_task.c
    ${APPLICATION_DIR}/gimbal/gimbal_yaw_pitch_direct.c
    ${APPLICATION_DIR}/gimbal/gimbal_virtual.c
    ${APPLICATION_DIR}/IMU/IMU_task.c
    ${APPLICATION_DIR}/IMU/IMU_solve.c
    ${APPLICATION_DIR}/mechanical_arm/mechanical_arm.c
    ${APPLICATION_DIR}/mechanical_arm/mechanical_arm_task.c
    ${APPLICATION_DIR}/mechanical_arm/mechanical_arm_penguin_mini.c
    ${APPLICATION_DIR}/mechanical_arm/mechanical_arm_engineer.c
    ${APPLICATION_DIR}/music/music_task.c
    ${APPLICATION_DIR}/music/music.c
    ${APPLICATION_DIR}/music/music_you.c
    ${APPLICATION_DIR}/music/music_unity.c
    ${APPLICATION_DIR}/music/music_canon.c
    ${APPLICATION_DIR}/music/music_castle_in_the_sky.c
    ${APPLICATION_DIR}/music/music_start.c
    ${APPLICATION_DIR}/music/music_referee.c
    ${APPLICATION_DIR}/music/music_error.c
    ${APPLICATION_DIR}/music/music_see_you_again.c
    ${APPLICATION_DIR}/music/music_hao_yun_lai.c
    ${APPLICATION_DIR}/music/music_meow.c
    ${APPLICATION_DIR}/music/music_gong_xi_fa_cai.c
    ${APPLICATION_DIR}/music/music_deja_vu.c
    ${APPLICATION_DIR}/music/music_motor_offline.c
    ${APPLICATION_DIR}/music/music_rc_offline.c
    ${APPLICATION_DIR}/other/led_flow_task.c
    ${APPLICATION_DIR}/other/oled_task.c
    ${APPLICATION_DIR}/other/remote_control.c
    ${APPLICATION_DIR}/other/test_task.c
    ${APPLICATION_DIR}/other/voltage_task.c
    ${APPLICATION_DIR}/referee/referee.c
    ${APPLICATION_DIR}/referee/referee_usart_task.c
    ${APPLICATION_DIR}/remote_control/ps2_task.c
    ${APPLICATION_DIR}/robot_cmd/CAN_receive.c
    ${APPLICATION_DIR}/robot_cmd/CAN_cmd_dji.c
    ${APPLICATION_DIR}/robot_cmd/CAN_cmd_cybergear.c
    ${APPLICATION_DIR}/robot_cmd/motor.c
    ${APPLICATION_DIR}/robot_cmd/CAN_cmd_damiao.c
    ${APPLICATION_DIR}/robot_cmd/CAN_cmd_lingkong.c
    ${APPLICATION_DIR}/robot_cmd/CAN_communication.c
    ${APPLICATION_DIR}/robot_cmd/SupCap.c
    ${APPLICATION_DIR}/robot_cmd/CAN_cmd_SupCap.c
    ${APPLICATION_DIR}/robot_cmd/PWM_cmd_pump.c
    ${APPLICATION_DIR}/robot_cmd/PWM_cmd_servo.c
    ${APPLICATION_DIR}/shoot/shoot.c
    ${APPLICATION_DIR}/shoot/shoot_task.c
    ${APPLICATION_DIR}/shoot/shoot_fric_trigger.c
)

add_executable(${PROJECT_NAME}.elf
    ${STARTUP_SOURCES}
    ${CUBEMX_SOURCES}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${USB_SOURCES}
    ${BSP_SOURCES}
    ${DEVICE_SOURCES}
    ${ALGORITHM_SOURCES}
    ${SUPPORT_SOURCES}
    ${APPLICATION_SOURCES}
)

target_link_libraries(${PROJECT_NAME}.elf)

set(LINKER_SCRIPT "${PROJECT_DIR}/cmake/standard_robot.ld")
set_property(TARGET ${PROJECT_NAME}.elf PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})
configure_file(${LINKER_SCRIPT} ${CMAKE_CURRENT_BINARY_DIR}/standard_robot.ld COPYONLY)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${INC_DIR}
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Inc
    ${DRIVERS_DIR}/STM32F4xx_HAL_Driver/Inc/Legacy
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/include
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${MIDDLEWARES_DIR}/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${DRIVERS_DIR}/CMSIS/Device/ST/STM32F4xx/Include
    ${DRIVERS_DIR}/CMSIS/Include
    ${APPLICATION_DIR}
    ${BSP_DIR}/boards
    ${COMPONENTS_DIR}/devices
    ${COMPONENTS_DIR}/algorithm
    ${COMPONENTS_DIR}/algorithm/Include
    ${COMPONENTS_DIR}/support
    ${APPLICATION_DIR}/protocol
    ${COMPONENTS_DIR}/controller
    ${MIDDLEWARES_DIR}/ST/STM32_USB_Device_Library/Core/Inc
    ${MIDDLEWARES_DIR}/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${APPLICATION_DIR}/assist
    ${APPLICATION_DIR}/chassis
    ${APPLICATION_DIR}/gimbal
    ${APPLICATION_DIR}/other
    ${APPLICATION_DIR}/referee
    ${APPLICATION_DIR}/remote_control
    ${APPLICATION_DIR}/robot_cmd
    ${APPLICATION_DIR}/shoot
    ${APPLICATION_DIR}/IMU
    ${APPLICATION_DIR}/mechanical_arm
    ${APPLICATION_DIR}/music
    ${APPLICATION_DIR}/custom_controller
    ${APPLICATION_DIR}/communication
    ${APPLICATION_DIR}/calibrate
    ${APPLICATION_DIR}/typedef
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    USE_HAL_DRIVER
    STM32F407xx
    ARM_MATH_CM4
    __FPU_USED=1U
    __FPU_PRESENT=1U
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Keil target: -O1 with debug information. The Debug build type maps to
# -Og (debugging-friendly) with full debug info for best variable visibility.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(C_OPT_FLAGS -Og -g3)
else()
    set(C_OPT_FLAGS -O1 -g3)
endif()

target_compile_options(${PROJECT_NAME}.elf PRIVATE
    ${DEVICE_FLAGS}
    $<$<COMPILE_LANGUAGE:C>:${C_OPT_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections>
    $<$<COMPILE_LANGUAGE:C>:-fdata-sections>
    $<$<COMPILE_LANGUAGE:C>:-Wall>
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${DEVICE_FLAGS}
    -Tstandard_robot.ld
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--no-warn-mismatch
    --specs=nano.specs
    --specs=nosys.specs
)

# Libraries must follow the object files on the link line. Raw relative names
# avoid both CMake path resolution and GNU ld non-ASCII path handling.
set_property(TARGET ${PROJECT_NAME}.elf APPEND PROPERTY LINK_LIBRARIES
    "-L."
    "-l:AHRS.lib"
    "-l:arm_cortexM4lf_math.lib"
    "-lm"
)

set(ELF_FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf)
set(BIN_FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(HEX_FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf
    BYPRODUCTS ${BIN_FILE} ${HEX_FILE}
    COMMENT "building firmware artifacts"
    VERBATIM
)
