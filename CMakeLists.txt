add_executable(accelerometer
    accelerometer.c
        )

# pull in common dependencies
target_link_libraries(accelerometer 
    pico_stdlib
    hardware_i2c)

pico_enable_stdio_usb(accelerometer 1)
pico_enable_stdio_uart(accelerometer 0)
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(accelerometer)


