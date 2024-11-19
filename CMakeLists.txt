add_executable(BarcodeDecoder
    irsensor.c
)

target_link_libraries(BarcodeDecoder
    pico_stdlib
    hardware_adc
    hardware_pwm
)

set(FREERTOS_KERNEL_PATH "C:/FreeRTOS") # Path to FreeRTOS kernel
message("FREERTOS_KERNEL_PATH: ${FREERTOS_KERNEL_PATH}") # Debugging purposes

# Add FreeRTOS Kernel path
include_directories(${FREERTOS_KERNEL_PATH}/include)
include_directories(${FREERTOS_KERNEL_PATH}/portable/GCC/ARM_CM0)

# Link FreeRTOS Kernel
target_sources(BarcodeDecoder PRIVATE
    ${FREERTOS_KERNEL_PATH}/Source/croutine.c
    ${FREERTOS_KERNEL_PATH}/Source/event_groups.c
    ${FREERTOS_KERNEL_PATH}/Source/list.c
    ${FREERTOS_KERNEL_PATH}/Source/queue.c
    ${FREERTOS_KERNEL_PATH}/Source/tasks.c
    ${FREERTOS_KERNEL_PATH}/Source/timers.c
    ${FREERTOS_KERNEL_PATH}/Source/portable/MemMang/heap_4.c
    ${FREERTOS_KERNEL_PATH}/Source/portable/GCC/ARM_CM0/port.c
)

target_compile_definitions(BarcodeDecoder PRIVATE
    configUSE_PREEMPTION=1
    configUSE_IDLE_HOOK=0
    configUSE_TICK_HOOK=0
)

pico_enable_stdio_usb(BarcodeDecoder 1)
pico_enable_stdio_uart(BarcodeDecoder 0)

pico_add_extra_outputs(BarcodeDecoder)