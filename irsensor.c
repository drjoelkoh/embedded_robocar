#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include <string.h>

#define PWM_PIN_1 2
#define DIR_PIN1_1 1
#define DIR_PIN2_1 0
#define PWM_PIN_2 5
#define DIR_PIN1_2 6
#define DIR_PIN2_2 4
#define LINE_SENSOR_PIN 26
#define BARCODE_SENSOR_PIN 27
#define MAX_PULSES 200
#define TOTAL_CHAR 43
#define DEBOUNCE_DELAY_US 20000
#define PULSE_COUNT_THRESHOLD 9

/* Barcode Data and Lookup Tables */
const char *barcode_lookup_chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-.$/+% ";
const char *barcode_patterns[] = {
    "001110101", "100100001", "001100001", "101100000", // 0-3
    "000110001", "100110000", "001110000", "000100101", // 4-7
    "100100100", "001100100", "100001001", "001001001", // 8-11
    "101001000", "000011001", "100011000", "001011000", // 12-15
    "000001101", "100001100", "001001100", "000011100", // 16-19
    "100000011", "001000011", "101000010", "000010011", // 20-23
    "100010010", "001010010", "000000111", "100000110", // 24-27
    "001000110", "000010110", "110000001", "011000001", // 28-31
    "111000000", "010010001", "110010000", "011010000", // 32-35
    "010000101", "110000100", "010101000", "010100010", // 36-39
    "010001010", "000101010", "011000100"               // 40-42
};

typedef struct {
    uint64_t pulses[MAX_PULSES];
    int count;
} PulseWidthVector;

PulseWidthVector pulse_vector;
volatile bool is_scanning_active = false;
uint64_t pulse_start = 0;
char scanned_binary_code[MAX_PULSES + 1] = "";

/* Initialize pulse width vector */
void init_pulse_width_vector(PulseWidthVector *vec) {
    vec->count = 0;
}

/* Add pulse width to vector */
bool add_pulse_width(PulseWidthVector *vec, uint32_t pulse_width) {
    if (vec->count >= MAX_PULSES) {
        printf("Error: Pulse vector is full\n");
        return false;
    }
    vec->pulses[vec->count++] = pulse_width;
    return true;
}

/* Calculate threshold for pulse normalization */
uint64_t calculate_threshold(const PulseWidthVector *vec) {
    uint64_t total_pulse = 0;
    for (int i = 0; i < vec->count; i++) {
        total_pulse += vec->pulses[i];
    }
    return total_pulse / vec->count;
}

/* Normalize pulse widths to binary string based on threshold */
void normalize_pulse_widths(const PulseWidthVector *vec, char *binary_code, uint64_t threshold) {
    for (int i = 0; i < vec->count; i++) {
        binary_code[i] = (vec->pulses[i] > threshold) ? '1' : '0';
    }
    binary_code[vec->count] = '\0';
}

/* Reverse a binary code string */
void reverse_binary_code(const char *input, char *output, int length) {
    for (int i = 0; i < length; i++) {
        output[i] = input[length - i - 1];
    }
    output[length] = '\0';
}

/* Match binary code against patterns in both original and reversed order */
const char* decode_from_patterns(const char *binary_code, int length) {
    for (int i = 0; i < TOTAL_CHAR; i++) {
        if (strncmp(binary_code, barcode_patterns[i], length) == 0) {
            return &barcode_lookup_chars[i];
        }
    }

    // Check reversed code
    char reversed_binary_code[MAX_PULSES + 1];
    reverse_binary_code(binary_code, reversed_binary_code, length);
    for (int i = 0; i < TOTAL_CHAR; i++) {
        if (strncmp(reversed_binary_code, barcode_patterns[i], length) == 0) {
            return &barcode_lookup_chars[i];
        }
    }

    return NULL;
}

/* Barcode sensor interrupt handler */
void barcode_sensor_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_interrupt_time = 0;
    uint32_t interrupt_time = time_us_64();

    if (interrupt_time - last_interrupt_time < DEBOUNCE_DELAY_US) {
        return;
    }
    last_interrupt_time = interrupt_time;

    if (gpio == BARCODE_SENSOR_PIN) {
        if (!is_scanning_active) {
            is_scanning_active = true;
            init_pulse_width_vector(&pulse_vector);
            pulse_start = time_us_64();
            printf("Scanning started.\n");
        } else {
            uint32_t pulse_width = time_us_64() - pulse_start;
            add_pulse_width(&pulse_vector, pulse_width);
            pulse_start = time_us_64();

            if (pulse_vector.count >= PULSE_COUNT_THRESHOLD) {
                uint64_t threshold = calculate_threshold(&pulse_vector);
                normalize_pulse_widths(&pulse_vector, scanned_binary_code, threshold);
                const char *decoded_char = decode_from_patterns(scanned_binary_code, PULSE_COUNT_THRESHOLD);
                
                if (decoded_char) {
                    printf("Decoded character: %c\n", *decoded_char);
                } else {
                    printf("Failed to decode barcode.\n");
                }
                is_scanning_active = false;
            }
        }
    }
}

/* Initialize GPIO for barcode sensor */
void init_gpio() {
    gpio_init(BARCODE_SENSOR_PIN);
    gpio_set_dir(BARCODE_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(BARCODE_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, barcode_sensor_irq_handler);
}

int main() {
    stdio_init_all();
    init_gpio();
    printf("Barcode Scanning Initialized\n");

    while (1) {
        tight_loop_contents();
    }
}
