#include <string.h>
#include <stdlib.h>
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/dhcp.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include <stdio.h>


//Wifi stuff
#define TCP_PORT 4242
#define BUF_SIZE 2048

// Define a message buffer structure to store different types of accelerometer commands
/* typedef struct {
    char turnleft[BUF_SIZE];  // Buffer to store "turn left"
    char turnright[BUF_SIZE]; // Buffer to store "turn right"
    char forward[BUF_SIZE];   // Buffer to store "move forward"
    char backward[BUF_SIZE];  // Buffer to store "move backward"
    char stop[BUF_SIZE];      // Buffer to store "stop"
} DirectionCommands; */

typedef struct {
    float forward_spd;
    float backward_spd;
    float turn_right_spd;
    float turn_left_spd;
    bool stop;
    bool auto_mode;
} DirectionCommands;



// pins for motor stuff
#define LEFT_MOTOR_PIN 2  //left wheel
#define RIGHT_MOTOR_PIN 3  //right wheel
#define DIR_PIN1 0
#define DIR_PIN2 1
#define DIR_PIN3 4
#define DIR_PIN4 5
//#define DIR_BTN 20
//#define LOWSPD_BTN 21
//#define HISPD_BTN 22
#define DEBOUNCE_DELAY_MS 100
#define pwm_freq 500
//#define target_speed 100
//#define desired_rpm 2400

float current_speed = 0;
bool is_clockwise = true;
bool turning_right = false;
bool is_moving = true;
bool line_following_mode = false;
bool auto_mode = false; //MAKE THIS FALSE TO START UP IN REMOTE CONTROL MODE
float target_speed = 60; //target
float desired_rpm_left = 1800;
float desired_rpm_right = 1800;
float prev_motor_speed = 0;
float pwm_clock = 0;
uint16_t wrap_value = 0;
MessageBufferHandle_t directionMessageBuffer;
MessageBufferHandle_t speedMessageBuffer; 
MessageBufferHandle_t objectDistanceMessageBuffer;
MessageBufferHandle_t leftWheelSpeedMessageBuffer;
MessageBufferHandle_t rightWheelSpeedMessageBuffer;
MessageBufferHandle_t directionControlMessageBuffer;
MessageBufferHandle_t barcodeMessageBuffer;

//ultrasonic and wheel encoder stuff
#define ULTRA_TRIG 7
#define ULTRA_ECHO 6
#define ROTARY_PIN_L 9 // not needed now rmb to change pin number
#define ROTARY_PIN_R 8
#define WHEEL_CIRCUMFERENCE 21.0 // in cm
#define ENCODER_CIRCUMFERENCE 9.0 // in cm
#define PULSES_PER_REV 20

int timeout = 26100;
volatile uint32_t global_pulse_count_left = 0;
volatile uint32_t global_pulse_count_for_turn_right = 0;
volatile uint32_t global_pulse_count_right = 0;
absolute_time_t last_pulse_time_left;  // tracks previous point in time that the timing pulse was generated
absolute_time_t last_pulse_time_right;
float last_printed_distance = -1.0;
float total_distance_travelled = 0.0;

#define PROXIMITY_DIST 17.0 // Distance threshold in cm for object detection
volatile bool pulse_started = false;
volatile uint32_t pulse_start_time = 0;
volatile uint32_t pulse_end_time = 0;
volatile bool object_detected = false;
volatile bool is_turning = false;
volatile bool is_stopping = false;
volatile bool stopped = false;
volatile bool on_black = false;
volatile bool motor_control_enabled = true;


//IR sensor stuff
#define LINE_SENSOR_PIN 26 // iswapped these
#define BARCODE_SENSOR_PIN 27
#define BLACK_THRESHOLD 165
#define line_follow_toggle_btn 20
#define MAX_PULSES 200
#define TOTAL_CHAR 43
#define DEBOUNCE_DELAY_US 20000
#define PULSE_COUNT_THRESHOLD 7


void init_motor_control();
void set_motor_direction(bool clockwise);
void set_motor_spd(int pin, float speed_percent);
void set_left_motor_spd(float speed_percent);
void set_right_motor_spd(float speed_percent);
void set_left_motor_direction(bool clockwise);
void set_right_motor_direction(bool clockwise);
void remote_forward(float speed);
void remote_backward(float speed);
void remote_turn_left(float speed);
void remote_turn_right(float speed);
void go_stop(float distance);
void turn_right_90(float speed);
float convertRPMToSpeed(float current_rpm, float desired_rpm);
void encoderPinInit();
float getLeftWheelRPM(float sample_time_ms);
float getRightWheelRPM(float sample_time_ms);
void wheel_speed_task(void *pvParameters);
void printWheelSpeedTask(void *pvParameters);
void ultSonicPinInit();
uint32_t echo_pulse(uint trigger_pin, uint echo_pin);
void updateDistanceTraveled();
void direction_controls (void *pvParameters);
void sendDirectioncontrolMessage(DirectionCommands *dir_commands, size_t len);
void turn_right(float speed);
void turn_left(float speed);
void stop();
void go(float speed);
void line_following();
void find_line();

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

static uint32_t last_interrupt_time = 0;
uint32_t interrupt_time = 0;

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




//Wifi stuff 
//A structure to hold the server's state and the connected clients
typedef struct {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *clients[3];  // Array to hold client PCBs
    uint8_t buffer_recv[BUF_SIZE];
    DirectionCommands dir_commands;  // Store the messages (e.g., turnleft, turnright)
} TCP_SERVER_T;


// Initialize the server's state
static TCP_SERVER_T *tcp_server_init(void) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        return NULL;
    }
    return state;
}

// Send the received message to all clients (Can use this function and edit it to send whatever message necessary)
static err_t tcp_server_send_to_clients(TCP_SERVER_T *state, const uint8_t *data, size_t len, struct tcp_pcb *sender) {
    for (int i = 0; i < 3; i++) {
        if (state->clients[i] != NULL && state->clients[i] != sender) {
            err_t err = tcp_write(state->clients[i], data, len, TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) {
                tcp_output(state->clients[i]);
            }
        }
    }
    return ERR_OK;
}

// Handle incoming data from clients
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;

    if (!p) {  // Client disconnected
        for (int i = 0; i < 3; i++) {
            if (state->clients[i] == tpcb) {
                state->clients[i] = NULL;  // Mark the slot as empty
                printf("Client disconnected\n");
                break;
            }
        }
        return ERR_OK;
    }

    if (p->tot_len > 0) {
        pbuf_copy_partial(p, state->buffer_recv, p->tot_len, 0);
        state->buffer_recv[p->tot_len] = '\0';  // Null-terminate the received string

        if (p->tot_len == sizeof(DirectionCommands)) {
            // Copy the received data directly into the state->dir_commands struct
            pbuf_copy_partial(p, &state->dir_commands, sizeof(DirectionCommands), 0);

            // Debug print each field
            printf("Received command:\n");
            printf("  Forward speed: %.2f\n", state->dir_commands.forward_spd);
            printf("  Backward speed: %.2f\n", state->dir_commands.backward_spd);
            printf("  Turn right speed: %.2f\n", state->dir_commands.turn_right_spd);
            printf("  Turn left speed: %.2f\n", state->dir_commands.turn_left_spd);
            printf("  Stop: %s\n", state->dir_commands.stop ? "True" : "False");
            printf("  Auto mode: %s\n", state->dir_commands.auto_mode ? "True" : "False");

            // Relay the received data to all clients except the sender
            // THIS FUNCTION SEND STUFF TO DASHBOARD
            tcp_server_send_to_clients(state, (uint8_t *)&state->dir_commands, sizeof(DirectionCommands), tpcb);
            sendDirectioncontrolMessage(&state->dir_commands, sizeof(state->dir_commands));
            //xMessageBufferSend(directionControlMessageBuffer, &state->dir_commands, sizeof(DirectionCommands), portMAX_DELAY);
        } else {
            printf("Received data size mismatch. Expected %zu bytes, got %u bytes.\n", sizeof(DirectionCommands), p->tot_len);
        }

        // Mark data as received
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);  // Free the received buffer
    return ERR_OK;
}

// Handle new client connections
static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;

    if (err != ERR_OK || client_pcb == NULL) {
        printf("Failed to accept client connection, error code: %d\n", err);
        return ERR_VAL;
    }

    // Try to find an empty slot for the new client
    for (int i = 0; i < 3; i++) {
        if (state->clients[i] == NULL) {
            state->clients[i] = client_pcb;
            printf("Client %d connected\n", i);

            tcp_arg(client_pcb, state);
            tcp_recv(client_pcb, tcp_server_recv);
            return ERR_OK;
        }
    }

    // No available slots, reject the connection
    printf("No available slots for new client, closing connection\n");
    tcp_close(client_pcb);
    return ERR_ABRT;
}

// Open the TCP server and start listening for connections
static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T *)arg;

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("Failed to create PCB\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        printf("Failed to bind to port %d, error code: %d\n", TCP_PORT, err);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 3);
    if (!state->server_pcb) {
        tcp_close(pcb);
        printf("Failed to listen on port %d\n", TCP_PORT);
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

// Function for the TCP server FreeRTOS task
void tcp_server_task(void *pvParameters) {
    TCP_SERVER_T *state = tcp_server_init();
    
    if (!state) {
        printf("Failed to initialize server\n");
        vTaskDelete(NULL); // End the task if initialization fails
    }

    if (!tcp_server_open(state)) {
        vTaskDelete(NULL); // End the task if opening the server fails
    }

    struct netif *netif = netif_list;
    ip_addr_t ip_addr = netif->ip_addr;
    // Print the IP address
    printf("Server IP Address: %s\n", ipaddr_ntoa(&ip_addr));
    printf("Auto mode: %d\n", auto_mode);

    // Main server loop
    while (true) {
        
        cyw43_arch_poll(); // Poll the WiFi stack
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait for 500 ms
    }

    printf("Server shutdown\n");
    vTaskDelete(NULL); // Cleanly exit the task
}

void sendDirectioncontrolMessage(DirectionCommands *dir_commands, size_t len) {
    size_t available_space = xMessageBufferSpaceAvailable(directionControlMessageBuffer);
    if (available_space >= len) {
        xMessageBufferSend(directionControlMessageBuffer, dir_commands, sizeof(DirectionCommands), portMAX_DELAY);
    } else {
        printf("Direction message buffer full, available space: %d, size of buffer,: %d\n", available_space, len);
        
    }
    
}

void direction_controls (void *pvParameters) {
    DirectionCommands dir_commands;
    while (true) {
        if (!auto_mode) {
            if (xMessageBufferReceive(directionControlMessageBuffer, &dir_commands, sizeof(dir_commands), portMAX_DELAY) > 0) {
                if (dir_commands.forward_spd > 0) {
                    dir_commands.forward_spd > 50 ? dir_commands.forward_spd = 50 : dir_commands.forward_spd;
                    remote_forward(50);
                    
                } else if (dir_commands.backward_spd > 0) {
                    dir_commands.backward_spd > 60 ? dir_commands.backward_spd = 60 : dir_commands.backward_spd;
                    remote_backward(50);
                    
                if (dir_commands.turn_right_spd > 0 && dir_commands.turn_right_spd > dir_commands.forward_spd) {
                        dir_commands.turn_right_spd += dir_commands.forward_spd/2;
                        dir_commands.turn_right_spd < 50 ? dir_commands.turn_right_spd = 50 : dir_commands.turn_right_spd;
                        remote_turn_right(50);
                        
                }
                if (dir_commands.turn_left_spd > 0 && dir_commands.turn_left_spd > dir_commands.forward_spd) {
                        dir_commands.turn_left_spd += dir_commands.forward_spd/2;
                        dir_commands.turn_left_spd < 50 ? dir_commands.turn_left_spd = 50 : dir_commands.turn_left_spd;
                        remote_turn_left(50);
                        
                }
                } if (dir_commands.stop) {
                    set_left_motor_spd(0);
                    set_right_motor_spd(0);
                    
                } if (dir_commands.auto_mode) {
                    auto_mode = !auto_mode;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

// initialize GPIO and PWM
void init_motor_control() {
    // Direction pins
    gpio_init(DIR_PIN1); gpio_set_dir(DIR_PIN1, GPIO_OUT);
    gpio_init(DIR_PIN2); gpio_set_dir(DIR_PIN2, GPIO_OUT);
    gpio_init(DIR_PIN3); gpio_set_dir(DIR_PIN3, GPIO_OUT);
    gpio_init(DIR_PIN4); gpio_set_dir(DIR_PIN4, GPIO_OUT);

    
    // PWM
    gpio_set_function(LEFT_MOTOR_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR_PIN, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(LEFT_MOTOR_PIN);
    uint slice_num2 = pwm_gpio_to_slice_num(RIGHT_MOTOR_PIN);

    pwm_clock = (float)clock_get_hz(clk_sys) / 64.0f;
    /* wrap_value = (uint16_t)(pwm_clock / pwm_freq) - 1; */
    wrap_value = 65535;
    float freq = 100.0f;
    float clock_freq = 125000000.0f;  // Default clock frequency of the Pico in Hz
    uint32_t divider = clock_freq / (freq * 65536);  // Compute clock divider
    pwm_set_clkdiv(slice_num1, divider); 
    pwm_set_wrap(slice_num1, wrap_value);
    pwm_set_clkdiv(slice_num2, divider); 
    pwm_set_wrap(slice_num2, wrap_value);
    pwm_set_enabled(slice_num1, true); 
    pwm_set_enabled(slice_num2, true);

   /*  gpio_init(DIR_BTN); 
    gpio_set_dir(DIR_BTN, GPIO_IN); 
    gpio_set_pulls(DIR_BTN, true, false); 
    gpio_init(LOWSPD_BTN); 
    gpio_set_dir(LOWSPD_BTN, GPIO_IN); 
    gpio_set_pulls(LOWSPD_BTN, true, false); 
    gpio_init(HISPD_BTN); 
    gpio_set_dir(HISPD_BTN, GPIO_IN); 
    gpio_set_pulls(HISPD_BTN, true, false);  */
}

void set_motor_direction(bool clockwise) {
    gpio_put(DIR_PIN1, !clockwise); gpio_put(DIR_PIN2, clockwise);
    gpio_put(DIR_PIN3, !clockwise); gpio_put(DIR_PIN4, clockwise);
    //printf("Direction set to %s\n", clockwise ? "Clockwise" : "Counterclockwise");
    
}

void set_left_motor_direction(bool clockwise) {
    gpio_put(DIR_PIN1, !clockwise); gpio_put(DIR_PIN2, clockwise);
}

void set_right_motor_direction(bool clockwise) {
    gpio_put(DIR_PIN3, !clockwise); gpio_put(DIR_PIN4, clockwise);
}

float convertRPMToSpeed(float current_rpm, float desired_rpm) {
    return (current_rpm / desired_rpm) * target_speed;
}

void remote_forward (float speed) {
    printf("[Remote] Moving forward\n");
    set_left_motor_direction(is_clockwise);
    set_right_motor_direction(is_clockwise);
    set_left_motor_spd(speed);
    set_right_motor_spd(speed);
}

void remote_backward (float speed) {
    printf("[Remote] Moving backward\n");
    set_left_motor_direction(!is_clockwise);
    set_right_motor_direction(!is_clockwise);
    set_left_motor_spd(speed);
    set_right_motor_spd(speed);
}

void remote_turn_left(float speed) {
    printf("[Remote] Turning left\n");
    set_left_motor_direction(!is_clockwise);
    set_right_motor_direction(is_clockwise);
    set_left_motor_spd(speed);
    set_right_motor_spd(speed);
}

void remote_turn_right(float speed) {
    printf("[Remote] Turning right\n");
    set_left_motor_direction(is_clockwise);
    set_right_motor_direction(!is_clockwise);
    set_left_motor_spd(speed);
    set_right_motor_spd(speed);
}


void set_motor_spd(int pin, float speed_percent) {
    if (speed_percent < 0) speed_percent = 0;

    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value);
    current_speed = speed_percent;
    pwm_set_gpio_level(pin, duty_cycle);
    if (speed_percent != prev_motor_speed) {
        //printf("[Motor] Set speed of pin %d to %.2f%%\n", pin, speed_percent);
        prev_motor_speed = speed_percent; // Update last printed speed
    }
}

void set_left_motor_spd(float speed_percent) {
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;
    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value);
    current_speed = speed_percent;
    pwm_set_gpio_level(LEFT_MOTOR_PIN, duty_cycle);
    if (speed_percent != prev_motor_speed) {
        //printf("[Motor] Set left speed to %.2f%%\n", speed_percent);
        prev_motor_speed = speed_percent; // Update last printed speed
    }
}

void set_right_motor_spd(float speed_percent) {
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;
    speed_percent += 7;
    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value);
    current_speed = speed_percent;
    pwm_set_gpio_level(RIGHT_MOTOR_PIN, duty_cycle);
    if (speed_percent != prev_motor_speed) {
        //printf("[Motor] Set right speed %d", speed_percent);
        prev_motor_speed = speed_percent; // Update last printed speed
    }
}

/* void set_motor_rpm(int pin, float rpm) {
    if (rpm > desired_rpm) rpm = desired_rpm;
    float speed = convertRPMToSpeed(rpm);
    set_motor_spd(pin, speed);
} */



void go_stop(float distance) {

    is_stopping = true;
    float current_distance = total_distance_travelled;
    float stop_at = current_distance + distance;
    printf("Current distance: %.2f cm\n, is_stopping at: %.2f cm\n", current_distance, stop_at);
    while (total_distance_travelled < stop_at) {
        // Optionally add some sleep to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    set_left_motor_spd(0);
    set_right_motor_spd(0);
    //set_motor_spd(LEFT_MOTOR_PIN, 0);
    //set_motor_spd(RIGHT_MOTOR_PIN, 0);
    is_moving = false;
    motor_control_enabled = false;
    stopped = true;
    printf("Stopped at %.2f cm\n", total_distance_travelled);
    /* while (true) {
        if (total_distance_travelled >= stop_at) {
            set_motor_spd(LEFT_MOTOR_PIN, 0);
            set_motor_spd(RIGHT_MOTOR_PIN, 0);
            is_moving = false;
            motor_control_enabled = false;
            stopped = true;
            printf("Stopped at %.2f cm\n", total_distance_travelled);
            break;
        }
    } */
}

void turn_right_90(float speed) {
    motor_control_enabled ? printf("Motor control enabled\n") : printf("Motor control disabled\n");
    is_turning = true;
    printf("Motor control set to disabled\n");
    float wheelbase = 6.3; // in cm. THis is the distance of the front wheel to the rear wheel axel. I need this to turn 90 degrees
    float turn_distance = (3.14159 * wheelbase) / 2.0; // turning radius for 90 degrees
    float distance_per_pulse = ENCODER_CIRCUMFERENCE / PULSES_PER_REV;
    uint32_t required_pulses = (turn_distance / distance_per_pulse) / 2.0;
    
    global_pulse_count_for_turn_right = 0; // Reset pulse count
    gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 0);
    gpio_put(DIR_PIN3, 0); gpio_put(DIR_PIN4, 0); 
    sleep_ms(1000);
    //set_left_motor_spd(speed);
    //set_right_motor_spd(speed);
    set_motor_spd(LEFT_MOTOR_PIN, speed);
    set_motor_spd(RIGHT_MOTOR_PIN, speed);
    gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 0);
    gpio_put(DIR_PIN3, 0); gpio_put(DIR_PIN4, 0); 
    //sleep_ms(1000);
    
    // Set directions for turn
    gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 1); // clockwise
    gpio_put(DIR_PIN3, 1); gpio_put(DIR_PIN4, 0); // anticlockwise
    printf("Required pulses: %ld\n", (long)required_pulses);
    while (global_pulse_count_for_turn_right < (required_pulses)) {
        // Wait until the pulses count reaches the required amount
        printf("Pulses: %ld\n", (long)global_pulse_count_for_turn_right);
        tight_loop_contents(); // Idle while waiting
    }
    
    // Stop motors after turn
    set_left_motor_spd(0);
    set_right_motor_spd(0);
    //set_motor_spd(LEFT_MOTOR_PIN, 0);
    //set_motor_spd(RIGHT_MOTOR_PIN, 0);
    printf("Completed 90-degree turn\n");
    //motor_control_enabled = true;
    is_turning = false;
    
}

/* void turn_right(bool turn_right, float speed) {
    if (turn_right) {
        gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 0);
        gpio_put(DIR_PIN3, 0); gpio_put(DIR_PIN4, 0); 
        sleep_ms(300);
        printf("Turning right\n");
        set_motor_spd(LEFT_MOTOR_PIN, speed);
        set_motor_spd(RIGHT_MOTOR_PIN, 0);
        gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 1); //clockwise
        gpio_put(DIR_PIN3, 1); gpio_put(DIR_PIN4, 0); //anticlockwise
        //set_motor_spd(LEFT_MOTOR_PIN, 100);
        //set_motor_spd(RIGHT_MOTOR_PIN, 50);
        is_turning = true;
        printf("Turning flag is now true\n");
        turn_right_90(speed);
        
    }
    else {
        
        set_motor_direction(is_clockwise);
        set_motor_spd(LEFT_MOTOR_PIN, 100);
        set_motor_spd(RIGHT_MOTOR_PIN, 100);
    }   
} */


// Task to handle motor speed based on button input
/* void speed_task(void *pvParameters) {
    while (true) {
        uint32_t msg;
        if (xMessageBufferReceive(speedMessageBuffer, &msg, sizeof(msg), portMAX_DELAY)) {
            if (msg == LOWSPD_BTN) {
                motor_speed = 50; // Change speed as desired
                set_motor_spd(LEFT_MOTOR_PIN, motor_speed);
                set_motor_spd(RIGHT_MOTOR_PIN, motor_speed);
                printf("[Motor] Speed set to LOW%%\n"); // Update log message
            } else if (msg == HISPD_BTN) {
                motor_speed = 100;
                set_motor_spd(LEFT_MOTOR_PIN, motor_speed);
                set_motor_spd(RIGHT_MOTOR_PIN, motor_speed);
                printf("[Motor] Speed set to HIGH%%\n");
            }
        }
    }
} */

/* Initialise the Photo Interrupt sensor input pin */
void encoderPinInit() {
    gpio_init(ROTARY_PIN_L);
    gpio_set_dir(ROTARY_PIN_L, GPIO_IN);
    gpio_pull_down(ROTARY_PIN_L); // (HIGH=1,LOW=0)
    
}

/* Calculates number of pulses divided by timeframe to get speed at point in time*/
float getRevsPerMin(uint32_t timeframe_pulse_counts, float duration_sec) {
    return (timeframe_pulse_counts / duration_sec) * 60.0f;
}

void updateDistanceTraveled() {
    // Calculate distance per pulse
    float distance_per_pulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REV; // in cm
    total_distance_travelled += global_pulse_count_left * distance_per_pulse; // Total distance
    global_pulse_count_left = 0; // Reset pulse count
    
}

/* Uses the photo interrupter sensor to measure the rotational speed of the encoder
(Final version  will require wheel circumference specs to gauge distance and speed of car)
ARGS: sample+time_ms: duration which the wheelspeed is measured over (ms)
1. Wait for the regularly timed pulse edge trigger to start the measurement
2. In the period of one pulse, count the number of times that the disc blocks the sensor (1-0 trigger)
3. Divide the disc_pulse_count value by the duration of one time_pulse 
4. 
5. [To be implemented later] Multiply rotational speed by car wheel circumference to get actual distance and speed*/
float getLeftWheelRPM(float sample_time_ms) {
    // measure the RPM at 
    absolute_time_t current_time;
    float duration_seconds, revs_per_min;

    sleep_ms(sample_time_ms); // 
    current_time = get_absolute_time();
    duration_seconds = absolute_time_diff_us(last_pulse_time_left, current_time) / 1e6;  //time difference in microseconds then convert into seconds
    revs_per_min = getRevsPerMin(global_pulse_count_left, duration_seconds);
    updateDistanceTraveled();
    //printf("[Encoder] Pulses: %ld",  (long)global_pulse_count_left);

    global_pulse_count_left = 0; // reset pulse count
    last_pulse_time_left = current_time;   // set 'new' time to reference previous time from
    return revs_per_min;
}

float getRightWheelRPM(float sample_time_ms) {
    // measure the RPM at 
    absolute_time_t current_time;
    float duration_seconds, revs_per_min;

    sleep_ms(sample_time_ms); // 
    current_time = get_absolute_time();
    duration_seconds = absolute_time_diff_us(last_pulse_time_right, current_time) / 1e6;  //time difference in microseconds then convert into seconds
    revs_per_min = getRevsPerMin(global_pulse_count_right, duration_seconds);
    updateDistanceTraveled();
    //printf("[Encoder] Pulses: %ld",  (long)global_pulse_count_left);

    global_pulse_count_right = 0; // reset pulse count
    last_pulse_time_right = current_time;   // set 'new' time to reference previous time from
    return revs_per_min;
}

void wheel_speed_task(void *pvParameters) {
    const TickType_t sampleTimeTicks = pdMS_TO_TICKS(1000);
    float previous_speed_L = 0;
    float previous_speed_R = 0;
    while (true) {
        float speedL = getLeftWheelRPM(100);
        if (speedL != previous_speed_L) {
            xMessageBufferSend(leftWheelSpeedMessageBuffer, &speedL, sizeof(speedL), portMAX_DELAY);
            previous_speed_L = speedL;
        }
        
        float speedR = getRightWheelRPM(100);
        if (speedR != previous_speed_R) {
            xMessageBufferSend(rightWheelSpeedMessageBuffer, &speedR, sizeof(speedR), portMAX_DELAY);
            previous_speed_R = speedR;
        }
        
        vTaskDelay(sampleTimeTicks); // Delay for the sampling period
    }
}
// Task to receive and print wheel speed
void printWheelSpeedTask(void *pvParameters) {
    float speed;
    while (true) {
        if (xMessageBufferReceive(leftWheelSpeedMessageBuffer, &speed, sizeof(speed), portMAX_DELAY) > 0) {
            printf("[Encoder] Left wheel rpm: %.2f RPM\n", speed);
        }
        if (xMessageBufferReceive(rightWheelSpeedMessageBuffer, &speed, sizeof(speed), portMAX_DELAY) > 0) {
            printf("[Encoder] Right wheel rpm: %.2f RPM\n", speed);
        }
    }
}

/* ULTRA SONIC PART*/
/* Initialise the Ultrasonic sensor trigger and echo pins*/
void ultSonicPinInit() {
    gpio_init(ULTRA_TRIG);
    gpio_init(ULTRA_ECHO);
    gpio_set_dir(ULTRA_TRIG, GPIO_OUT); // Pin sends the pulse of sound wave out
    gpio_set_dir(ULTRA_ECHO, GPIO_IN);  // Pin receives the data of incoming reflected sound wave
}

/* Return time difference between when echo_pin goes from 1 to 0*/
uint32_t echo_pulse(uint trigger_pin, uint echo_pin) {
    gpio_put(trigger_pin, 1);   //sends out a HIGH signal from trigger_pin
    sleep_us(10);
    gpio_put(trigger_pin, 0);   // waits a duration of 10us before going back to LOW

    uint64_t width = 0;
    
    while (gpio_get(echo_pin) == 0) tight_loop_contents(); // wait until echo_pin == 1 (receive echo)
    uint32_t startTime = time_us_32();    // get time when pin goes HIGH
    while (gpio_get(echo_pin) == 1)// wait until echo_pin goes back to 0 implement interrupt for timing instead of blocking
    {
        width++;
        sleep_us(1);    // wait 1 microsecond
        if (width > timeout) return 0;
    }
    uint32_t endTime = time_us_32();  // get the time when pin goes LOW
    uint32_t time_diff = endTime - startTime;
    return time_diff;
}

float getCm(uint trigPin, uint echoPin) {
    uint32_t pulseLength = echo_pulse(trigPin, echoPin);
    float distance = (pulseLength / 2.0 ) * 0.0343;    // speed of sound (echo) is 34300 cm/s
    return distance;
}

// PID control function for left motor
float pid_control_left(float current_rpm_left, float dt) {
    //float Kp = 0.8, Ki = 0.1, Kd = 0.4; // PID coefficients
    float Kp = 1.0, Ki = 0.4, Kd = 0;
    float previous_error_left = 0.0;
    float integral_left = 0.0;
    // Convert current RPM to motor speed percentage
    float current_speed_left = convertRPMToSpeed(current_rpm_left, desired_rpm_left);
    
    // Calculate error between desired and actual speed
    float error = target_speed - current_speed_left; // We want the speed to reach target_speed
    integral_left += error * dt;
    float derivative = (error - previous_error_left) / dt;

    // Calculate PID output
    float output = Kp * error + Ki * integral_left + Kd * derivative;
    previous_error_left = error;

    //printf("Adjusted left motor speed: %.2f\n", output);    

    return output;
}

// PID control function for right motor
float pid_control_right(float current_rpm_right, float dt) {
    //float Kp = 1.1, Ki = 0.1, Kd = 0.51; // PID coefficients
    //float Kp = 2.6, Ki = 0.01, Kd = 0.035; //backup
    float Kp = 2.6, Ki = 0.01, Kd = 0.035;
    float previous_error_right = 0.0;
    float integral_right = 0.0;
    // Convert current RPM to motor speed percentage
    float current_speed_right = convertRPMToSpeed(current_rpm_right, desired_rpm_right);
    
    // Calculate error between desired and actual speed
    float error = target_speed - current_speed_right; // We want the speed to reach target_speed
    integral_right += error * dt;
    float derivative = (error - previous_error_right) / dt;

    // Calculate PID output
    float output = Kp * error + Ki * integral_right + Kd * derivative;
    previous_error_right = error;

    //printf("Adjusted right motor speed: %.2f\n", output);

    return output;
}

void update_motor_speeds(float left_rpm, float right_rpm, float dt) {
    // Get PID outputs for both motors
    float left_pid_output = pid_control_left(left_rpm, dt);
    float right_pid_output = pid_control_right(right_rpm, dt);

    // Line-following logic for turning
    if (on_black) {
        // Gradually reduce speed of one motor for turning
        //float turn_factor = 0.5;  // Adjust this for sharper or smoother turns
        //left_pid_output *= turn_factor; // Reduce left motor speed
        //right_pid_output *= turn_factor; // Reduce right motor speed
        set_left_motor_spd(40);  // Sharp turn: left motor slows down
        set_right_motor_spd(70);
    }

    float left_motor_speed = left_pid_output;
    float right_motor_speed = right_pid_output;

    // Limit motor speed to 100%
    if (left_motor_speed > 100) left_motor_speed = 100;
    if (right_motor_speed > 100) right_motor_speed = 100;
    if (left_motor_speed < 0) left_motor_speed = 0;
    if (right_motor_speed < 0) right_motor_speed = 0;
    //printf("New Left motor speed: %.2f, New Right motor speed: %.2f\n", left_motor_speed, right_motor_speed);

    // Set motor speeds
    
    set_left_motor_spd(left_motor_speed);
    set_right_motor_spd(right_motor_speed);
    
    //set_motor_spd(LEFT_MOTOR_PIN, left_motor_speed);
    //set_motor_spd(RIGHT_MOTOR_PIN, right_motor_speed);
}

void motor_control_task(void *pvParameters) {
    absolute_time_t last_time = get_absolute_time();
    
    while (true) {
        if (auto_mode && !is_turning) {
            // Get current RPM from encoders (you need to implement these functions)
            float left_rpm = getLeftWheelRPM(50);  // Example: measure left wheel RPM over 100ms
            float right_rpm = getRightWheelRPM(50);  // Example: measure right wheel RPM over 100ms

            // Calculate the time difference (dt) between iterations
            absolute_time_t current_time = get_absolute_time();
            float dt = absolute_time_diff_us(last_time, current_time) / 1e6;  // in seconds
            last_time = current_time;

            // Adjust motor speeds based on PID controller output
            update_motor_speeds(left_rpm, right_rpm, dt);

            // Delay for a short time to avoid busy-waiting
              // Adjust the delay as necessary
        } else {
            
            continue;
            
            
        }
        
    }
}


void isr_handler(uint gpio, uint32_t events) {
    switch (gpio) {
        case ULTRA_ECHO:
            if (events & GPIO_IRQ_EDGE_RISE) {
                pulse_start_time = time_us_32();
                pulse_started = true;
            } else if (events & GPIO_IRQ_EDGE_FALL && pulse_started) {
                pulse_end_time = time_us_32();
                uint32_t pulse_duration = pulse_end_time - pulse_start_time;
                pulse_started = false;
                // Calculate distance in cm
                float distance = (pulse_duration / 2.0) * 0.0343;
                if (distance <= PROXIMITY_DIST && distance > 1.0) {
                    object_detected = true;
                } else {
                    object_detected = false;
                }
            }
            break;
        case ROTARY_PIN_R:
            global_pulse_count_right++;
            break;
        case ROTARY_PIN_L:
            global_pulse_count_left++;
            //global_pulse_count_for_turn_right++;
            break;
        case BARCODE_SENSOR_PIN:
            interrupt_time = time_us_64();

            if (interrupt_time - last_interrupt_time < DEBOUNCE_DELAY_US) {
                return;
            }
            last_interrupt_time = interrupt_time;

            if (!is_scanning_active) {
                is_scanning_active = true;
                init_pulse_width_vector(&pulse_vector);
                pulse_start = time_us_64();
            } else {
                uint32_t pulse_width = time_us_64() - pulse_start;
                add_pulse_width(&pulse_vector, pulse_width);
                pulse_start = time_us_64();

                if (pulse_vector.count >= PULSE_COUNT_THRESHOLD) {
                    uint64_t threshold = calculate_threshold(&pulse_vector);
                    normalize_pulse_widths(&pulse_vector, scanned_binary_code, threshold);
                    const char *decoded_char = decode_from_patterns(scanned_binary_code, PULSE_COUNT_THRESHOLD);
                    
                    if (decoded_char) {
                        /* printf("Decoded character: %c\n", *decoded_char); */
                        xMessageBufferSend(barcodeMessageBuffer, decoded_char, sizeof(*decoded_char), portMAX_DELAY);
                    }
                    is_scanning_active = false;
                }
            break;
        }
    }
}

void print_barcode_task(void *pvParameters) {
    char barcode_char;
    char prev_char;
    while (true) {
        if (xMessageBufferReceive(barcodeMessageBuffer, &barcode_char, sizeof(barcode_char), portMAX_DELAY) > 0) {
            if (barcode_char != prev_char) {
                printf("[Barcode] Decoded character: %c\n", barcode_char);
                prev_char = barcode_char;
            }
        }
    }
}


// Send trigger pulse to ultrasonic sensor
void send_trigger_pulse() {
    gpio_put(ULTRA_TRIG, 1);
    sleep_us(10); // Pulse duration
    gpio_put(ULTRA_TRIG, 0);
}

// Task to monitor distance and turn if necessary
void auto_mode_task(void *pvParameters) {
    bool in_cooldown = false;

    while (true) {
        if (auto_mode) {
            if (!in_cooldown && !stopped && !on_black) {

                if (is_moving) {
                    set_motor_direction(is_clockwise);
                    //set_left_motor_spd(60);
                    //set_right_motor_spd(60);
                    set_motor_spd(LEFT_MOTOR_PIN, 60);
                    set_motor_spd(RIGHT_MOTOR_PIN, 60); 
                }
                if (stopped) {
                    set_left_motor_spd(0);
                    set_right_motor_spd(0);
                    //set_motor_spd(LEFT_MOTOR_PIN, 0);
                    //set_motor_spd(RIGHT_MOTOR_PIN, 0);
                }
                    
            }
            } else {
                // Cooldown period to avoid continuous triggering
                vTaskDelay(pdMS_TO_TICKS(500));  // Cooldown duration
                in_cooldown = false;
            }
        }
    
        vTaskDelay(pdMS_TO_TICKS(100));
}

void proximitiy_stop(void *pvParameters) {
    while (true) {
        float distance = getCm(ULTRA_TRIG, ULTRA_ECHO);
        if (object_detected) {
            printf("Distance from object: %.2f cm\n", distance);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void print_dist_task(void *pvParameters) {
    float distance;
    while (true) {
        // Receive the distance from the message buffer
        if (xMessageBufferReceive(objectDistanceMessageBuffer, &distance, sizeof(distance), portMAX_DELAY) > 0) {
            // Check if the distance is different from the last printed value
            if (distance != last_printed_distance) {
                //printf("[Ultrasonic] Distance from object: %.2f cm\n", distance);
                last_printed_distance = distance;
            }
            
        }
    }
}

void go(float speed){
    auto_mode = true;
    is_moving = true;
    stopped = false;
    set_motor_direction(is_clockwise);
    set_motor_spd(LEFT_MOTOR_PIN, speed);
    set_motor_spd(RIGHT_MOTOR_PIN, speed);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void turn_right(float speed) {
    is_turning = true;
    set_left_motor_direction(is_clockwise);
    set_right_motor_direction(!is_clockwise);
    set_motor_spd(LEFT_MOTOR_PIN, speed);
    set_motor_spd(RIGHT_MOTOR_PIN, speed);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void turn_left(float speed) {
    is_turning = true;
    set_left_motor_direction(!is_clockwise);
    set_right_motor_direction(is_clockwise);
    set_motor_spd(LEFT_MOTOR_PIN, speed);
    set_motor_spd(RIGHT_MOTOR_PIN, speed);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void stop() {
    auto_mode = false;
    set_left_motor_spd(0);
    set_right_motor_spd(0);
    stopped = true;
    //set_motor_spd(LEFT_MOTOR_PIN, 0);
    //set_motor_spd(RIGHT_MOTOR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void ir_sensor_init() {
    /* adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(0); */
    gpio_init(LINE_SENSOR_PIN);
    gpio_set_dir(LINE_SENSOR_PIN, GPIO_IN);
    gpio_pull_down(LINE_SENSOR_PIN);

    gpio_init(BARCODE_SENSOR_PIN);
    gpio_set_dir(BARCODE_SENSOR_PIN, GPIO_IN);
    gpio_pull_down(BARCODE_SENSOR_PIN);

    gpio_init(line_follow_toggle_btn);
    gpio_set_dir(line_follow_toggle_btn, GPIO_IN);
    gpio_pull_down(line_follow_toggle_btn);
}

/* void line_following_task(void *pvParameters) {
    while (true) {
        uint32_t ir_value = adc_read();
    
        if (gpio_get(line_follow_toggle_btn) == 0) {
            line_following_mode = !line_following_mode;
            printf("Line following mode: %s\n", line_following_mode ? "Enabled" : "Disabled");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (line_following_mode) {
            stop();
            if (ir_value < BLACK_THRESHOLD) {
                
                go();
                printf("Line detected, IR sensor value: %d\n", ir_value);
            
            } else {
               
                printf("No line detected, IR sensor value: %d\n", ir_value);
                stop();
                sleep_ms(500);
                turn_right();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
} */

void line_following_task(void *pvParameters) {
    while (true) {
        if (gpio_get(line_follow_toggle_btn) == 0) {
            line_following_mode = !line_following_mode;
            printf("Line following mode: %s\n", line_following_mode ? "Enabled" : "Disabled");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (line_following_mode) {
            if (gpio_get(LINE_SENSOR_PIN) == 0) {
                //printf("WHITE\n");
                on_black = false;
                target_speed = 45; // Set target speed for straight movement
            } else {
                //printf("BLACK\n");
                on_black = true;
                target_speed = 40; // Optionally lower speed slightly when turning
                // TODO: ttry adding a delay to stop it from changing to white too quickly so that it can turn more
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); //TODO: try lower or remove this to make it respond faster
    }
}

void find_line(int num_of_tries) {
    float speed = 45;
    float turn_duration = 300;
    while (num_of_tries > 0) {
        turn_right(speed);
        if(gpio_get(LINE_SENSOR_PIN) == 1) {
            printf("Line found\n");
            go(45);
            break;
        }
        sleep_ms(turn_duration);
        stop();
        sleep_ms(500);
        turn_left(speed);
        if(gpio_get(LINE_SENSOR_PIN) == 1) {
            printf("Line found\n");
            go(45);
            break;
        }
        turn_duration = turn_duration * 1.5;
        sleep_ms(turn_duration);
        stop();
        sleep_ms(500);
        speed = speed + 10;
        num_of_tries--;
    }
    
}

// Main function
int main() {
    stdio_init_all();
    init_motor_control();
    ultSonicPinInit();
    encoderPinInit();
    ir_sensor_init();
     // Initialize hardware and WiFi architecture
    if (cyw43_arch_init()) {
        printf("WiFi initialization failed!\n");
        return 1;
    }
    
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("WiFi connection failed\n");
        return 1;
    }

    // Create message buffers for direction and speed
    directionMessageBuffer = xMessageBufferCreate(64);
    if (directionMessageBuffer == NULL) {
        printf("Direction message buffer creation failed!\n");
        return 1;
    }
    
    speedMessageBuffer = xMessageBufferCreate(64);
    if (speedMessageBuffer == NULL) {
        printf("Speed message buffer creation failed!\n");
        return 1;
    }

    objectDistanceMessageBuffer = xMessageBufferCreate(64);
    if (objectDistanceMessageBuffer == NULL) {
        printf("Object distance message buffer creation failed!\n");
        return 1;
    }

    leftWheelSpeedMessageBuffer = xMessageBufferCreate(64);
    if (leftWheelSpeedMessageBuffer == NULL) {
        printf("Wheel speed message buffer creation failed!\n");
        return 1;
    }

    rightWheelSpeedMessageBuffer = xMessageBufferCreate(64);
    if (rightWheelSpeedMessageBuffer == NULL) {
        printf("Wheel speed message buffer creation failed!\n");
        return 1;
    }

    directionControlMessageBuffer = xMessageBufferCreate(2048);
    if (directionControlMessageBuffer == NULL) {
        printf("Direction control message buffer creation failed!\n");
        return 1;
    }

    barcodeMessageBuffer = xMessageBufferCreate(64);
    if (barcodeMessageBuffer == NULL) {
        printf("Barcode message buffer creation failed!\n");
        return 1;
    }


    // Set up interrupts
    /* gpio_set_irq_enabled_with_callback(DIR_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(LOWSPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(HISPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr); */
    
    gpio_set_irq_enabled_with_callback(ROTARY_PIN_L, GPIO_IRQ_EDGE_RISE, true, &isr_handler);
    gpio_set_irq_enabled_with_callback(ROTARY_PIN_R, GPIO_IRQ_EDGE_RISE, true, &isr_handler);
    gpio_set_irq_enabled_with_callback(ULTRA_ECHO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &isr_handler);
    gpio_set_irq_enabled_with_callback(BARCODE_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, &isr_handler);

    // Create the TCP server task
    xTaskCreate(tcp_server_task, "TCP Server Task", 512, NULL, 1, NULL);
    /* xTaskCreate(direction_task, "Direction Task", 512, NULL, 1, NULL);
    xTaskCreate(speed_task, "Speed Task", 512, NULL, 1, NULL); */
    xTaskCreate(auto_mode_task, "Auto Task", 512, NULL, 1, NULL);
    xTaskCreate(print_dist_task, "Print Distance Task", 512, NULL, 1, NULL);
    xTaskCreate(wheel_speed_task, "Wheel Speed Task", 512, NULL, 1, NULL);
    xTaskCreate(printWheelSpeedTask, "Print Wheel Speed Task", 512, NULL, 1, NULL);
    xTaskCreate(motor_control_task, "Motor Control Task", 512, NULL, 1, NULL);
    xTaskCreate(line_following_task, "Line Following Task", 512, NULL, 1, NULL);
    xTaskCreate(direction_controls, "Direction Controls Task", 512, NULL, 1, NULL);
    xTaskCreate(proximitiy_stop, "Proximity Stop Task", 512, NULL, 1, NULL);
    xTaskCreate(print_barcode_task, "Print Barcode Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();

  // Deinitialize WiFi if the scheduler stops 
    cyw43_arch_deinit();
    return 0;
}
