#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

// Define GPIO pins and constants
#define ADC_PIN 6
#define DIGITAL_PIN 7
#define WHITE_THRESHOLD 300         // ADC threshold for detecting white surface
#define TIME_THRESHOLD 1000000      // 1 second threshold in microseconds for thick/thin lines
#define MAX_BARCODE_LENGTH 30       // Maximum number of pulses for barcode storage

// Variables for pulse width timing
absolute_time_t pulse_start, pulse_end;
bool black_line_detected = false;
bool barcode_decoding = false;  // Flag to indicate barcode decoding
bool on_black_line = true;      // Track whether currently on a black line

// Array to store barcode elements with max capacity 30
char barcode_elements[MAX_BARCODE_LENGTH];
int element_count = 0;

// Function to initialize ADC and GPIO pins
void init_ir_sensor() {
    adc_init();
    adc_gpio_init(ADC_PIN);           // Initialize ADC pin (GP26)
    gpio_init(DIGITAL_PIN);           // Initialize digital pin (GP27)
    gpio_set_dir(DIGITAL_PIN, GPIO_IN); // Set digital pin as input
}

// Function to check surface type based on GP26 and GP27
bool is_white_surface_detected() {
    return !gpio_get(ADC_PIN) && !gpio_get(DIGITAL_PIN);
}

// Function to calculate pulse width and classify it
void measure_pulse_width_and_classify(bool from_black_to_white) {
    pulse_end = get_absolute_time();
    int64_t pulse_width = absolute_time_diff_us(pulse_start, pulse_end);

    if (element_count < MAX_BARCODE_LENGTH) {
        if (from_black_to_white) {
            barcode_elements[element_count++] = (pulse_width < TIME_THRESHOLD) ? 'B' : 'T'; 
            printf("Black Line Detected. Pulse Width (us): %lld\n", pulse_width);
        } else {
            barcode_elements[element_count++] = (pulse_width < TIME_THRESHOLD) ? 'S' : 'W'; 
            printf("White Space Detected. Pulse Width (us): %lld\n", pulse_width);
        }
    } else {
        printf("Max barcode storage capacity reached. Unable to store more elements.\n");
    }

    pulse_start = pulse_end;  // Reset start time for the next pulse
}

// Function to display all stored elements on demand
void display_barcode_elements() {
    printf("Stored Barcode Elements: ");
    for (int i = 0; i < element_count; i++) {
        printf("%c ", barcode_elements[i]);
    }
    printf("\n");
}

// Function to continuously detect surfaces and measure pulse widths
void detect_surface_and_measure_pulse() {
    bool white_surface_detected = is_white_surface_detected();

    if (white_surface_detected) {
        if (black_line_detected) {  // Transition from black line to white surface
            measure_pulse_width_and_classify(true);
            black_line_detected = false;
            on_black_line = false;

            if (barcode_decoding) {
                display_barcode_elements();  // Print stored barcode elements
                element_count = 0;           // Reset element count
                barcode_decoding = false;    // Reset decoding flag
            }
        }
        //printf("White surface detected!\n");
    } else {
        if (!black_line_detected) {  // Transition from white surface to black line
            measure_pulse_width_and_classify(false);
            black_line_detected = true;
            on_black_line = true;
            display_barcode_elements();

            if (!barcode_decoding) {
                printf("Barcode start detected!\n");
                barcode_decoding = true;
                element_count = 0;  // Reset for a new barcode
            }
        }
        //printf("Black line detected!\n");
    }
}

// Main function
int main() {
    stdio_init_all();               // Initialize standard I/O
    init_ir_sensor();               // Initialize IR sensor

    pulse_start = get_absolute_time();  // Initialize pulse start time

    while (true) {
        detect_surface_and_measure_pulse();  // Continuously detect line and measure pulse
        sleep_ms(10);                       // Reduced delay for increased sensitivity

        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            display_barcode_elements();  // Display stored elements if input detected
        }
    }
}
