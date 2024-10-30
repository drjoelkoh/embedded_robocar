#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"

// Define GPIO pins and constants for two sensors
#define SENSOR1_DIGITAL_PIN 3
#define SENSOR2_DIGITAL_PIN 27
#define TIME_THRESHOLD 1000000      // 1 second threshold in microseconds for thick/thin lines
#define MAX_BARCODE_LENGTH 30       // Maximum number of pulses for barcode storage

// Variables for pulse width timing
absolute_time_t pulse_start, pulse_end;
bool black_line_detected1 = false;
bool black_line_detected2 = false;
bool barcode_decoding1 = false;
bool barcode_decoding2 = false;

// String to store barcode elements
char barcode_elements[MAX_BARCODE_LENGTH + 1];  // +1 for null-terminator
int element_count = 0;

// Function to initialize GPIO pins for two sensors
void init_ir_sensors() {
    gpio_init(SENSOR1_DIGITAL_PIN);           
    gpio_set_dir(SENSOR1_DIGITAL_PIN, GPIO_IN);
    gpio_init(SENSOR2_DIGITAL_PIN);           
    gpio_set_dir(SENSOR2_DIGITAL_PIN, GPIO_IN);
}

// Function to detect surface transitions for each sensor
bool is_white_surface_detected(int sensor_pin) {
    return !gpio_get(sensor_pin); // Assuming active low for white surface
}

// Function to measure pulse width and classify it, appending to the barcode string
void measure_pulse_width_and_classify(bool from_black_to_white, int sensor_id) {
    pulse_end = get_absolute_time();
    int64_t pulse_width = absolute_time_diff_us(pulse_start, pulse_end);

    if (element_count < MAX_BARCODE_LENGTH) {
        char element = '\0';
        if (from_black_to_white) {
            element = (pulse_width < TIME_THRESHOLD) ? 'B' : 'T';
        } else {
            element = (pulse_width < TIME_THRESHOLD) ? 'S' : 'W';
        }

        barcode_elements[element_count++] = element;  // Add element to barcode
        barcode_elements[element_count] = '\0';       // Null-terminate the string

        // Print the newly added element and the entire barcode string
        printf("Added Element: %c\n", element);
        printf("Current Barcode Elements: %s\n", barcode_elements);
    } else {
        printf("Max barcode storage capacity reached.\n");
    }

    pulse_start = pulse_end;  // Reset start time for the next pulse
}

// Function to continuously detect surfaces and measure pulse widths for each sensor
void detect_surface_and_measure_pulse() {
    bool white_surface_detected1 = is_white_surface_detected(SENSOR1_DIGITAL_PIN);
    bool white_surface_detected2 = is_white_surface_detected(SENSOR2_DIGITAL_PIN);

    // Sensor 1
    if (white_surface_detected1) {
        if (black_line_detected1) {  // Transition from black line to white surface
            measure_pulse_width_and_classify(true, 1);
            black_line_detected1 = false;
            barcode_decoding1 = true;
        }
    } else {
        if (!black_line_detected1) {  // Transition from white surface to black line
            measure_pulse_width_and_classify(false, 1);
            black_line_detected1 = true;
            barcode_decoding1 = true;
        }
    }

    // Sensor 2
    if (white_surface_detected2) {
        if (black_line_detected2) {  // Transition from black line to white surface
            measure_pulse_width_and_classify(true, 2);
            black_line_detected2 = false;
            barcode_decoding2 = true;
        }
    } else {
        if (!black_line_detected2) {  // Transition from white surface to black line
            measure_pulse_width_and_classify(false, 2);
            black_line_detected2 = true;
            barcode_decoding2 = true;
        }
    }
}

// Main function
int main() {
    stdio_init_all();               // Initialize standard I/O
    init_ir_sensors();              // Initialize IR sensors

    pulse_start = get_absolute_time();  // Initialize pulse start time

    while (true) {
        detect_surface_and_measure_pulse();  // Continuously detect line and measure pulse
        sleep_ms(10);                       // Reduced delay for increased sensitivity

        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            printf("Barcode Elements: %s\n", barcode_elements);  // Display stored elements if input detected
        }
    }
}
