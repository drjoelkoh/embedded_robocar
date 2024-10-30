add_executable(wheel_speed
        wheel_speed.c
        )

target_include_directories(wheel_speed PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../..)

target_link_libraries(wheel_speed
        hardware_adc
        hardware_pwm
        hardware_gpio 
        hardware_timer 
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )
pico_add_extra_outputs(wheel_speed)
pico_enable_stdio_usb(wheel_speed 1)
