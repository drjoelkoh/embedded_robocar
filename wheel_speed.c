#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include <stdio.h>

// pins for motor stuff
#define PWM_PIN1 2
#define PWM_PIN2 3
#define DIR_PIN1 0
#define DIR_PIN2 1
#define DIR_PIN3 4
#define DIR_PIN4 5
#define DIR_BTN 20
#define LOWSPD_BTN 21
#define HISPD_BTN 22
#define DEBOUNCE_DELAY_MS 100
#define pwm_freq 500
#define MAX_SPEED 100
#define MAX_RPM 2400


bool is_clockwise = true;
bool turning_right = false;
bool is_moving = true;
float motor_speed = 100;
float prev_motor_speed = 0;
float pwm_clock = 0;
uint16_t wrap_value = 0;
MessageBufferHandle_t directionMessageBuffer;
MessageBufferHandle_t speedMessageBuffer; 
MessageBufferHandle_t objectDistanceMessageBuffer;
MessageBufferHandle_t leftWheelSpeedMessageBuffer;
MessageBufferHandle_t rightWheelSpeedMessageBuffer;

//ultrasonic and ir sensor stuff
#define ULTRA_TRIG 7
#define ULTRA_ECHO 6
#define ROTARY_PIN_L 16 // not needed now rmb to change pin number
#define ROTARY_PIN_R 8
#define WHEEL_CIRCUMFERENCE 21.0 // in cm
#define ENCODER_CIRCUMFERENCE 9.0 // in cm
#define PULSES_PER_REV 20

int timeout = 26100;
volatile uint32_t global_pulse_count_left = 0;
volatile uint32_t global_pulse_count_right = 0;
absolute_time_t last_pulse_time_left;  // tracks previous point in time that the timing pulse was generated
absolute_time_t last_pulse_time_right;
float last_printed_distance = -1.0;
float total_distance_travelled = 0.0;
float target_rpm = 2300;

#define DESIRED_DISTANCE 20.0 // Distance threshold in cm for object detection
volatile bool pulse_started = false;
volatile uint32_t pulse_start_time = 0;
volatile uint32_t pulse_end_time = 0;
volatile bool object_detected = false;
volatile bool turn_in_progress = false;
volatile bool stopping = false;
volatile bool stopped = false;
volatile bool motor_control_enabled = true;


// initialize GPIO and PWM
void init_motor_control() {
    // Direction pins
    gpio_init(DIR_PIN1); gpio_set_dir(DIR_PIN1, GPIO_OUT);
    gpio_init(DIR_PIN2); gpio_set_dir(DIR_PIN2, GPIO_OUT);
    gpio_init(DIR_PIN3); gpio_set_dir(DIR_PIN3, GPIO_OUT);
    gpio_init(DIR_PIN4); gpio_set_dir(DIR_PIN4, GPIO_OUT);

    // PWM
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN1);
    uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN2);

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

    gpio_init(DIR_BTN); 
    gpio_set_dir(DIR_BTN, GPIO_IN); 
    gpio_set_pulls(DIR_BTN, true, false); 
    gpio_init(LOWSPD_BTN); 
    gpio_set_dir(LOWSPD_BTN, GPIO_IN); 
    gpio_set_pulls(LOWSPD_BTN, true, false); 
    gpio_init(HISPD_BTN); 
    gpio_set_dir(HISPD_BTN, GPIO_IN); 
    gpio_set_pulls(HISPD_BTN, true, false); 
}

void set_motor_direction(bool clockwise) {
    gpio_put(DIR_PIN1, !clockwise); gpio_put(DIR_PIN2, clockwise);
    gpio_put(DIR_PIN3, !clockwise); gpio_put(DIR_PIN4, clockwise);
    //printf("Direction set to %s\n", clockwise ? "Clockwise" : "Counterclockwise");
    
}

float convertRPMToSpeed(float rpm) {
    return (rpm / MAX_RPM) * MAX_SPEED;
}

void set_motor_spd(int pin, float speed_percent) {
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;

    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value); 
    pwm_set_gpio_level(pin, duty_cycle);
    if (speed_percent != prev_motor_speed) {
        //printf("[Motor] Set speed of pin %d to %.2f%%\n", pin, speed_percent);
        prev_motor_speed = speed_percent; // Update last printed speed
    }
}

void set_motor_rpm(int pin, float rpm) {
    if (rpm > MAX_RPM) rpm = MAX_RPM;
    float speed = convertRPMToSpeed(rpm);
    set_motor_spd(pin, speed);
}



void go_stop(float distance) {
    float current_distance = total_distance_travelled;
    float stop_at = current_distance + distance;
    printf("Current distance: %.2f cm\n, Stopping at: %.2f cm\n", current_distance, stop_at);
    while (total_distance_travelled < stop_at) {
        // Optionally add some sleep to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    set_motor_spd(PWM_PIN1, 0);
    set_motor_spd(PWM_PIN2, 0);
    is_moving = false;
    motor_control_enabled = false;
    stopped = true;
    printf("Stopped at %.2f cm\n", total_distance_travelled);
    /* while (true) {
        if (total_distance_travelled >= stop_at) {
            set_motor_spd(PWM_PIN1, 0);
            set_motor_spd(PWM_PIN2, 0);
            is_moving = false;
            motor_control_enabled = false;
            stopped = true;
            printf("Stopped at %.2f cm\n", total_distance_travelled);
            break;
        }
    } */
}

void turn_right_90(float speed) {
    motor_control_enabled = false;
    float wheelbase = 9.0; // in cm. THis is the distance of the front wheel to the rear wheel axel. I need this to turn 90 degrees
    float turn_distance = (3.14159 * wheelbase) / 2.0; // turning radius for 90 degrees
    float distance_per_pulse = ENCODER_CIRCUMFERENCE / PULSES_PER_REV;
    uint32_t required_pulses = (turn_distance / distance_per_pulse) / 2.0;

    global_pulse_count_left = 0; // Reset pulse count
    set_motor_spd(PWM_PIN1, speed);
    set_motor_spd(PWM_PIN2, 0);
    gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 0);
    gpio_put(DIR_PIN3, 0); gpio_put(DIR_PIN4, 0); 
    sleep_ms(500);
    
    // Set directions for turn
    gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 1); // clockwise
    gpio_put(DIR_PIN3, 1); gpio_put(DIR_PIN4, 0); // anticlockwise
    
    while (global_pulse_count_left < (required_pulses)) {
        // Wait until the pulses count reaches the required amount
        //printf("Pulses: %ld\n", (long)global_pulse_count_left);
        tight_loop_contents(); // Idle while waiting
    }
    
    // Stop motors after turn
    set_motor_spd(PWM_PIN1, 0);
    set_motor_spd(PWM_PIN2, 0);
    printf("Completed 90-degree turn\n");
    motor_control_enabled = true;
    
}

/* void turn_right(bool turn_right, float speed) {
    if (turn_right) {
        gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 0);
        gpio_put(DIR_PIN3, 0); gpio_put(DIR_PIN4, 0); 
        sleep_ms(300);
        printf("Turning right\n");
        set_motor_spd(PWM_PIN1, speed);
        set_motor_spd(PWM_PIN2, 0);
        gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 1); //clockwise
        gpio_put(DIR_PIN3, 1); gpio_put(DIR_PIN4, 0); //anticlockwise
        //set_motor_spd(PWM_PIN1, 100);
        //set_motor_spd(PWM_PIN2, 50);
        turn_in_progress = true;
        printf("Turning flag is now true\n");
        turn_right_90(speed);
        
    }
    else {
        
        set_motor_direction(is_clockwise);
        set_motor_spd(PWM_PIN1, 100);
        set_motor_spd(PWM_PIN2, 100);
    }   
} */


// Task to handle motor speed based on button input
/* void speed_task(void *pvParameters) {
    while (true) {
        uint32_t msg;
        if (xMessageBufferReceive(speedMessageBuffer, &msg, sizeof(msg), portMAX_DELAY)) {
            if (msg == LOWSPD_BTN) {
                motor_speed = 50; // Change speed as desired
                set_motor_spd(PWM_PIN1, motor_speed);
                set_motor_spd(PWM_PIN2, motor_speed);
                printf("[Motor] Speed set to LOW%%\n"); // Update log message
            } else if (msg == HISPD_BTN) {
                motor_speed = 100;
                set_motor_spd(PWM_PIN1, motor_speed);
                set_motor_spd(PWM_PIN2, motor_speed);
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
    while (true) {
        float speedL = getLeftWheelRPM(100);  
        xMessageBufferSend(leftWheelSpeedMessageBuffer, &speedL, sizeof(speedL), portMAX_DELAY);
        float speedR = getRightWheelRPM(100);
        xMessageBufferSend(rightWheelSpeedMessageBuffer, &speedR, sizeof(speedR), portMAX_DELAY);
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



float pid_turn_speed(float current_distance) {
    // pid controller
    float Kp = 1.0;
    float Ki = 0.5;
    float Kd = 0.1;
    float dt = 0.5;
    float desired_distance = 10.0; // desired distance in cm
    float prev_error = 0.0, integral = 0.0;
    float error = desired_distance - current_distance;
    integral += error * dt; // Accumulate error over time
    float derivative = (error - prev_error) / dt; // Rate of change of error
    prev_error = error; // Update for next iteration

    // Apply PID formula, adjust gains as needed
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Limit output to avoid sudden changes
    if (output > 100.0) output = 100.0;
    if (output < 0.0) output = 0.0;

    if (output < 50) output += 50; // Minimum speed to prevent stalling

    return output;
}


void ultrasonic_wheel_speed_isr(uint gpio, uint32_t events) {
    if (gpio == ROTARY_PIN_L) {
        global_pulse_count_left++;
    } else if (gpio == ROTARY_PIN_R) {
        global_pulse_count_right++;
    }   else if (gpio == ULTRA_ECHO) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            pulse_start_time = time_us_32();
            pulse_started = true;
        } else if (events & GPIO_IRQ_EDGE_FALL && pulse_started) {
            pulse_end_time = time_us_32();
            uint32_t pulse_duration = pulse_end_time - pulse_start_time;
            pulse_started = false;

            // Calculate distance in cm
            float distance = (pulse_duration / 2.0) * 0.0343;
            if (distance <= DESIRED_DISTANCE && distance > 1.0) {
                object_detected = true;
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
void ultrasonic_task(void *pvParameters) {
    bool in_cooldown = false;

    while (true) {
        if (!in_cooldown && !stopped) {
            // Send a trigger pulse
            float distance = getCm(ULTRA_TRIG, ULTRA_ECHO);
            xMessageBufferSend(objectDistanceMessageBuffer, &distance, sizeof(distance), portMAX_DELAY);

            // Check if an object is detected
            if (object_detected && !turn_in_progress) {
                printf("[Ultrasonic] Object detected at %.2f cm\n . Turning right...\n", distance);
                float turn_speed = pid_turn_speed(distance);
                // Call the turn_right function to initiate a right turn
                /* turn_right(true, turn_speed); */
                turn_right_90(turn_speed);

                // Reset the object_detected flag and start cooldown
                object_detected = false;
                turn_in_progress = false;
                printf("Turning flag is now false\n");
                in_cooldown = true;
                set_motor_direction(is_clockwise);
                set_motor_spd(PWM_PIN1, 100);
                set_motor_spd(PWM_PIN2, 100); 
                go_stop(90.0);
                stopping = true;
                // Delay to prevent immediate re-triggering
                vTaskDelay(pdMS_TO_TICKS(2000));  // Adjust the delay as needed
                printf("Cooldown period started\n");

            } else {
                if (is_moving) {
                    set_motor_direction(is_clockwise);
                    set_motor_spd(PWM_PIN1, 100);
                    set_motor_spd(PWM_PIN2, 100); 
                }
                if (stopped) {
                    set_motor_spd(PWM_PIN1, 0);
                    set_motor_spd(PWM_PIN2, 0);
                }
                // If no object is detected, continue moving forward
                /* set_motor_direction(is_clockwise);
                set_motor_spd(PWM_PIN1, 100);
                set_motor_spd(PWM_PIN2, 100);  */
                
            }
        } else {
            // Cooldown period to avoid continuous triggering
            vTaskDelay(pdMS_TO_TICKS(1000));  // Cooldown duration
            in_cooldown = false;
        }

        
        vTaskDelay(pdMS_TO_TICKS(100));
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
                last_printed_distance = distance; // Update last printed distance
            }
        }
    }
}

// Main function
int main() {
    stdio_init_all();
    init_motor_control();
    ultSonicPinInit();
    encoderPinInit();

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

    // Set up interrupts
    /* gpio_set_irq_enabled_with_callback(DIR_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(LOWSPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(HISPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr); */
    
    gpio_set_irq_enabled_with_callback(ROTARY_PIN_L, GPIO_IRQ_EDGE_RISE, true, &ultrasonic_wheel_speed_isr);
    gpio_set_irq_enabled_with_callback(ROTARY_PIN_R, GPIO_IRQ_EDGE_RISE, true, &ultrasonic_wheel_speed_isr);
    gpio_set_irq_enabled_with_callback(ULTRA_ECHO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &ultrasonic_wheel_speed_isr);

    /* xTaskCreate(direction_task, "Direction Task", 512, NULL, 1, NULL);
    xTaskCreate(speed_task, "Speed Task", 512, NULL, 1, NULL); */
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 512, NULL, 1, NULL);
    xTaskCreate(print_dist_task, "Print Distance Task", 512, NULL, 1, NULL);
    xTaskCreate(wheel_speed_task, "Wheel Speed Task", 512, NULL, 1, NULL);
    xTaskCreate(printWheelSpeedTask, "Print Wheel Speed Task", 512, NULL, 1, NULL);
    
    //xTaskCreate(pid_speed_balancer, "PID Speed Balancer Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}
