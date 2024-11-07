#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <string.h>

// Motor and sensor pins
#define PWM_PIN_1 2                // GP2 for PWM Motor 1
#define DIR_PIN1_1 1               // GP0 for direction Motor 1
#define DIR_PIN2_1 0               // GP1 for direction Motor 1
#define PWM_PIN_2 5                // GP5 for PWM Motor 2
#define DIR_PIN1_2 6               // GP6 for direction Motor 2
#define DIR_PIN2_2 4               // GP4 for direction Motor 2
#define LINE_SENSOR_PIN 26         // GP26 for line following IR sensor
#define BARCODE_SENSOR_PIN 27      // GP27 for barcode scanning IR sensor

#define MAX_PULSES 100  // Maximum number of barcode pulses to store
#define DEBOUNCE_DELAY_MS 50  // Minimum time between interrupts to avoid multiple triggers

// Global variables
volatile bool is_scanning_active = false;
uint64_t last_interrupt_time = 0;
uint64_t pulse_start = 0;  // Pulse start time
uint64_t timing_for_bars[MAX_PULSES] = {0};
uint16_t num_bars_scanned = 0;

// Barcode lookup arrays
char barcode_lookup_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ_. $/+ %"; // Barcode lookup characters
const uint64_t barcode_lookup_bin[] = {
    0b000110100, 0b100100001, 0b001100001, 0b101100000, 0b000110001, 0b100110000, 0b001110000, 0b000100101, 
    0b100100100, 0b001100100, 0b100001001, 0b001001001, 0b101001000, 0b000011001, 0b100011000, 0b001011000, 
    0b000001101, 0b100001100, 0b001001100, 0b000011100, 0b100000011, 0b001000011, 0b101000010, 0b000010011, 
    0b100010010, 0b001010010, 0b000000111, 0b100000110, 0b001000110, 0b000010110, 0b110000001, 0b011000001, 
    0b111000000, 0b010010001, 0b110010000, 0b011010000, 0b010000101, 0b110000100, 0b010101000, 0b010100010, 
    0b010001010, 0b000101010, 0b011000100
};

// Initialize the pulse width vector
typedef struct {
    uint64_t pulses[MAX_PULSES];  // Array to store pulse widths
    int count;                    // Number of pulses stored
} PulseWidthVector;

PulseWidthVector pulse_vector;

void init_pulse_width_vector(PulseWidthVector *vec) {
    vec->count = 0;
}

bool add_pulse_width(PulseWidthVector *vec, uint32_t pulse_width) {
    if (vec->count >= MAX_PULSES) {
        return false; // Max reached, cannot add more pulses
    }
    vec->pulses[vec->count++] = pulse_width; // Add pulse width and increment counter
    return true;
}

// Normalize pulse widths by finding the min and max pulse lengths, setting a  threshold and converting pulse widths to binary values
void normalize_pulse_widths(const PulseWidthVector *vec, uint64_t *binary_code) {
    uint64_t min_pulse = UINT64_MAX;
    uint64_t max_pulse = 0;

    // Find the min and max pulse widths
    for (int i = 0; i < vec->count; i++) {
        if (vec->pulses[i] < min_pulse) min_pulse = vec->pulses[i];
        if (vec->pulses[i] > max_pulse) max_pulse = vec->pulses[i];
    }

    // Calculate average threshold and convert pulses to binary
    uint64_t threshold = (min_pulse + max_pulse) / 2;
    for (int i = 0; i < vec->count; i++) {
        binary_code[i] = (vec->pulses[i] > threshold) ? 1 : 0;
    }
}

// the pulse vector = buffer in C to store pulses
char decode_barcode(PulseWidthVector *pulse_vector) {
    uint64_t scanned_binary_code[MAX_PULSES] = {0}; // Storess binary version of pulse widths
    normalize_pulse_widths(pulse_vector, scanned_binary_code); 

    // Directly compare the binary code to the barcode lookup
    uint64_t scanned_code = 0;
    for (int i = 0; i < pulse_vector->count; i++) {
        scanned_code = (scanned_code << 1) | scanned_binary_code[i]; // Append bits
    }

    // Compare scanned code with lookup table
    for (int i = 0; i < sizeof(barcode_lookup_bin) / sizeof(barcode_lookup_bin[0]); i++) {
        if (scanned_code == barcode_lookup_bin[i]) {
            printf("Decoded character: %c\n", barcode_lookup_chars[i]);
            return barcode_lookup_chars[i];
        }
    }

    printf("Invalid barcode scanned!\n");
    return '\0'; // Return null character if no match
}

// Interrupt Handler - handles interrupts from barcode sensor
// Records pulse durations and initiates decoding when a sufficient number of pulses are collected
void barcode_sensor_irq_handler(uint gpio, uint32_t events) {
    uint32_t interrupt_time = to_ms_since_boot(get_absolute_time()); // Get the current time in milliseconds since the system started in milliseconds

    // Debounce: Ignore if interrupt time is too close to the last interrupt (incase its) 
    if (interrupt_time - last_interrupt_time < DEBOUNCE_DELAY_MS) {
        return;
    }
    last_interrupt_time = interrupt_time; // Update the last interrupt time

    if (gpio == BARCODE_SENSOR_PIN) { // Checks if the interrupt is from the barcode sensor
        if (!is_scanning_active) { // If it is not actively scanning... 
            // Start scanning
            is_scanning_active = true;
            init_pulse_width_vector(&pulse_vector);
            pulse_start = time_us_64();
            printf("Scanning started.\n");
        } else { // If it is actively scanning
            // Capture pulse width and add to vector
            uint32_t pulse_width = time_us_64() - pulse_start;
            add_pulse_width(&pulse_vector, pulse_width);
            pulse_start = time_us_64();  // Reset pulse start time for next pulse

            // Check if enough pulses are captured for decoding
            if (pulse_vector.count >= 9) { // Threshold for decoding
                is_scanning_active = false;
                printf("Scanning stopped. Decoding...\n");
                decode_barcode(&pulse_vector);
                init_pulse_width_vector(&pulse_vector);  // Reset pulse vector
            }
        }
    }
}

void line_following() {
    bool is_black = gpio_get(LINE_SENSOR_PIN); // Read line sensor
    if (is_black) {
        // Move forward
        gpio_put(DIR_PIN1_1, 1);
        gpio_put(DIR_PIN2_1, 0);
        gpio_put(DIR_PIN1_2, 1);
        gpio_put(DIR_PIN2_2, 0);
    } else {
        // Move backward
        gpio_put(DIR_PIN1_1, 0);
        gpio_put(DIR_PIN2_1, 1);
        gpio_put(DIR_PIN1_2, 0);
        gpio_put(DIR_PIN2_2, 1);
    }
}

void loop() {
    line_following();  // Follow the line
    sleep_ms(10);      // Short delay for responsiveness
}

int main() {
    stdio_init_all();

    // Motor control pins
    gpio_init(DIR_PIN1_1);
    gpio_init(DIR_PIN2_1);
    gpio_set_dir(DIR_PIN1_1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2_1, GPIO_OUT);
    gpio_init(DIR_PIN1_2);
    gpio_init(DIR_PIN2_2);
    gpio_set_dir(DIR_PIN1_2, GPIO_OUT);
    gpio_set_dir(DIR_PIN2_2, GPIO_OUT);

    // Line sensor pin
    gpio_init(LINE_SENSOR_PIN);
    gpio_set_dir(LINE_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(LINE_SENSOR_PIN);

    // Barcode sensor pin
    gpio_init(BARCODE_SENSOR_PIN);
    gpio_set_dir(BARCODE_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(BARCODE_SENSOR_PIN);

    // Setup barcode sensor interrupt
    gpio_set_irq_enabled_with_callback(BARCODE_SENSOR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &barcode_sensor_irq_handler);

    // Main loop
    while (true) {
        loop();
    }

    return 0;
}
