add_executable(task_5 task_5.c)

# pull in common dependencies
target_link_libraries(task_5 
        pico_stdlib 
        hardware_gpio 
        hardware_timer 
        hardware_adc)

pico_add_extra_outputs(task_5)

example_auto_set_url(task_5)

pico_enable_stdio_usb(task_5 1)