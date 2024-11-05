#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define ADC_PIN 26       // ADC pin for reading the barcode signal
#define THRESHOLD 512    // Set an appropriate threshold for HIGH state (0-4095)
#define MIN_NARROW 300   // Minimum pulse width for narrow bar/space in milliseconds
#define MIN_WIDE 600     // Minimum pulse width for wide bar/space in milliseconds
#define MAX_SYMBOLS 12   // Adjust based on maximum symbols in the barcode
#define BUTTON_20 20     // Button for narrow input
#define BUTTON_21 21     // Button for wide input
#define GP22_PIN 22      // GPIO pin for reading barcode signal

typedef enum {NARROW, WIDE} BarWidth;
typedef struct {
    BarWidth bars[9]; // Code 39 encodes each character as 9 bars and spaces
} Code39Symbol;

const Code39Symbol code39_symbols[] = {
    {{NARROW, NARROW, WIDE, WIDE, NARROW, NARROW, NARROW, WIDE, NARROW}}, // 'A'
    {{WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, WIDE, NARROW, NARROW}}, // 'B'
    {{NARROW, WIDE, NARROW, NARROW, NARROW, WIDE, WIDE, NARROW, NARROW}}, // 'C'
    {{NARROW, NARROW, NARROW, WIDE, WIDE, NARROW, NARROW, NARROW, WIDE}}, // 'D'
    {{WIDE, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, NARROW}}, // 'E'
    {{NARROW, WIDE, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW}}, // 'F'
    {{NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, WIDE, WIDE, NARROW}}, // 'G'
    {{WIDE, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW}}, // 'H'
    {{NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW}}, // 'I'
    {{NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, WIDE, NARROW, NARROW}}, // 'J'
    {{NARROW, NARROW, WIDE, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW}}, // 'K'
    {{WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW}}, // 'L'
    {{NARROW, WIDE, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, WIDE}}, // 'M'
    {{NARROW, NARROW, WIDE, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW}}, // 'N'
    {{NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, WIDE, WIDE}}, // 'O'
    {{WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, WIDE}}, // 'P'
    {{NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, WIDE, NARROW}}, // 'Q'
    {{NARROW, NARROW, WIDE, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW}}, // 'R'
    {{NARROW, NARROW, NARROW, WIDE, NARROW, WIDE, NARROW, NARROW, NARROW}}, // 'S'
    {{WIDE, NARROW, NARROW, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW}}, // 'T'
    {{NARROW, NARROW, WIDE, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW}}, // 'U'
    {{WIDE, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, NARROW}}, // 'V'
    {{NARROW, WIDE, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW}}, // 'W'
    {{NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, WIDE, WIDE, NARROW}}, // 'X'
    {{WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW}}, // 'Y'
    {{NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, WIDE, NARROW}}, // 'Z'
    {{NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, NARROW, WIDE}}, // '0'
    {{WIDE, NARROW, NARROW, NARROW, NARROW, NARROW, NARROW, WIDE, NARROW}}, // '1'
    {{NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, NARROW, NARROW, NARROW}}, // '2'
    {{NARROW, NARROW, WIDE, WIDE, NARROW, NARROW, NARROW, NARROW, NARROW}}, // '3'
    {{WIDE, NARROW, NARROW, NARROW, NARROW, NARROW, NARROW, NARROW, WIDE}}, // '4'
    {{NARROW, WIDE, NARROW, NARROW, NARROW, NARROW, WIDE, WIDE, WIDE}}, // '5'
    {{NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW}}, // '6'
    {{WIDE, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW}}, // '7'
    {{NARROW, WIDE, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, WIDE}}, // '8'
    {{NARROW, NARROW, WIDE, WIDE, WIDE, NARROW, NARROW, NARROW, NARROW}}, // '9'
    {{NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, WIDE, WIDE, NARROW}}, // '-' (dash)
    {{NARROW, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, NARROW, NARROW}}, // '.' (period)
    {{NARROW, NARROW, NARROW, NARROW, NARROW, WIDE, NARROW, NARROW, WIDE}}, // ' ' (space)
    {{WIDE, WIDE, NARROW, NARROW, NARROW, NARROW, NARROW, NARROW, WIDE}}, // '*' (start/stop character)
};

const char code39_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-. *"; // Characters corresponding to code39_symbols above

void decode_barcode(uint32_t *durations, int count) {
    printf("Starting barcode decoding with %d durations.\n", count);
    int symbol_index = 0;
    char decoded_message[MAX_SYMBOLS + 1] = {0};

    for (int i = 0; i < count; i += 9) {
        if (i + 9 > count) break; // Ensure there are 9 elements for each character

        Code39Symbol observed;
        for (int j = 0; j < 9; ++j) {
            observed.bars[j] = (durations[i + j] >= MIN_WIDE) ? WIDE : NARROW;
        }

        // Log observed pattern
        printf("Observed pattern: ");
        for (int j = 0; j < 9; ++j) {
            printf("%s ", (observed.bars[j] == NARROW) ? "NARROW" : "WIDE");
        }
        printf("\n");

        // Match observed pattern with known symbols
        for (int k = 0; k < sizeof(code39_symbols) / sizeof(code39_symbols[0]); ++k) {
            int match = 1;
            for (int j = 0; j < 9; ++j) {
                if (observed.bars[j] != code39_symbols[k].bars[j]) {
                    match = 0;
                    break;
                }
            }
            if (match) {
                decoded_message[symbol_index++] = code39_chars[k];
                printf("Matched symbol: %c\n", code39_chars[k]);
                break;
            }
        }
    }

    printf("Decoded Message: %s\n", decoded_message);
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0); // Select ADC input channel

    // Initialize buttons with pull-up resistors
    gpio_init(BUTTON_20);
    gpio_init(BUTTON_21);
    gpio_set_dir(BUTTON_20, GPIO_IN);
    gpio_set_dir(BUTTON_21, GPIO_IN);
    gpio_pull_up(BUTTON_20); // Enable pull-up resistor
    gpio_pull_up(BUTTON_21); // Enable pull-up resistor

    // Initialize GPIO pin for reading barcode signal
    gpio_init(GP22_PIN);
    gpio_set_dir(GP22_PIN, GPIO_IN);
    gpio_pull_up(GP22_PIN); // Enable pull-up resistor

    uint32_t durations[100]; // Increase array size if needed
    int count = 0;
    bool in_pulse = false;
    uint32_t pulse_start = 0;

    bool last_button_20_state = true; // Assume button is not pressed initially
    bool last_button_21_state = true; // Assume button is not pressed initially

    while (true) {
        // Read the barcode signal from GP22
        bool barcode_signal = gpio_get(GP22_PIN);
        uint32_t current_time = to_ms_since_boot(get_absolute_time()); // Current time in ms

        // Read button states
        bool current_button_20_state = gpio_get(BUTTON_20);
        bool current_button_21_state = gpio_get(BUTTON_21);

        // Check button 20 (NARROW)
        if (!current_button_20_state && last_button_20_state) { // Transition from not pressed to pressed
            durations[count++] = MIN_NARROW; // Manually add a NARROW bar
            printf("Manual input NARROW added: %d ms\n", MIN_NARROW);
            sleep_ms(200); // Simple debounce delay
        }

        // Check button 21 (WIDE)
        if (!current_button_21_state && last_button_21_state) { // Transition from not pressed to pressed
            durations[count++] = MIN_WIDE; // Manually add a WIDE bar
            printf("Manual input WIDE added: %d ms\n", MIN_WIDE);
            sleep_ms(200); // Simple debounce delay
        }

        // Check for barcode signal change
        if (barcode_signal && !in_pulse) {
            // Pulse started
            in_pulse = true;
            pulse_start = current_time;
            printf("Pulse started at %d ms\n", pulse_start);
        } else if (!barcode_signal && in_pulse) {
            // Pulse ended
            in_pulse = false;
            uint32_t duration = current_time - pulse_start;
            printf("Pulse ended, duration: %d ms\n", duration);

            if (duration > MIN_NARROW) {
                durations[count++] = duration;
                // Determine if narrow or wide
                if (duration >= MIN_WIDE) {
                    printf("Detected WIDE: %d ms\n", duration);
                } else {
                    printf("Detected NARROW: %d ms\n", duration);
                }
                printf("Duration added: %d ms (count: %d)\n", duration, count);
            }

            if (count >= 100) {
                // Reset count if it exceeds the array size
                count = 0;
                printf("Duration count reset due to overflow.\n");
            }

            // Decode the barcode if enough data has been collected
            if (count >= 9) {
                decode_barcode(durations, count);
                count = 0; // Reset count after decoding
            }
        }

        // Update last button states
        last_button_20_state = current_button_20_state;
        last_button_21_state = current_button_21_state;
    }

    return 0;
}
