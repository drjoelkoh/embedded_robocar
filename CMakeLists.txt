# Add the executable target for our project
add_executable(ir_sensor_proj ir_sensor_proj.c)

# Link the Pico SDK libraries
target_link_libraries(ir_sensor_proj pico_stdlib hardware_adc hardware_gpio hardware_timer)

# Enable USB output (if using the serial monitor via USB)
pico_enable_stdio_usb(ir_sensor_proj 1)

# Enable UART output (if using UART for debugging)
pico_enable_stdio_uart(ir_sensor_proj 0)

# Create map/bin/hex files
pico_add_extra_outputs(ir_sensor_proj)