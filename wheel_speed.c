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
#define pwm_freq 1000
//#define MAX_SPEED 100
//#define desired_rpm 2400

float current_speed = 0;
bool is_clockwise = true;
bool turning_right = false;
bool is_moving = true;
bool line_following_mode = false;
float max_speed = 70; //target
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

#define DESIRED_DISTANCE 17.0 // Distance threshold in cm for object detection
volatile bool pulse_started = false;
volatile uint32_t pulse_start_time = 0;
volatile uint32_t pulse_end_time = 0;
volatile bool object_detected = false;
volatile bool is_turning = false;
volatile bool is_stopping = false;
volatile bool stopped = false;
volatile bool motor_control_enabled = true;

//IR sensor stuff
#define IR_SENSOR_PIN_L 26
#define IR_SENSOR_PIN_R 27
#define line_follow_toggle_btn 20


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
    return (current_rpm / desired_rpm) * max_speed;
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

    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value);
    current_speed = speed_percent;
    pwm_set_gpio_level(LEFT_MOTOR_PIN, duty_cycle);
    if (speed_percent != prev_motor_speed) {
        //printf("[Motor] Set speed of pin %d to %.2f%%\n", pin, speed_percent);
        prev_motor_speed = speed_percent; // Update last printed speed
    }
}

void set_right_motor_spd(float speed_percent) {
    if (speed_percent < 0) speed_percent = 0;
    speed_percent += 7;
    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value);
    current_speed = speed_percent;
    pwm_set_gpio_level(RIGHT_MOTOR_PIN, duty_cycle);
    if (speed_percent != prev_motor_speed) {
        //printf("[Motor] Set speed of pin %d to %.2f%%\n", pin, speed_percent);
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

// PID control function for left motor
float pid_control_left(float current_rpm_left, float dt) {
    //float Kp = 0.8, Ki = 0.1, Kd = 0.4; // PID coefficients
    float Kp = 1.0, Ki = 0.4, Kd = 0;
    float previous_error_left = 0.0;
    float integral_left = 0.0;
    // Convert current RPM to motor speed percentage
    float current_speed_left = convertRPMToSpeed(current_rpm_left, desired_rpm_left);
    
    // Calculate error between desired and actual speed
    float error = max_speed - current_speed_left; // We want the speed to reach max_speed
    integral_left += error * dt;
    float derivative = (error - previous_error_left) / dt;

    // Calculate PID output
    float output = Kp * error + Ki * integral_left + Kd * derivative;
    previous_error_left = error;

    printf("Adjusted left motor speed: %.2f\n", output);    

    return output;
}

// PID control function for right motor
float pid_control_right(float current_rpm_right, float dt) {
    //float Kp = 1.1, Ki = 0.1, Kd = 0.51; // PID coefficients
    float Kp = 2.6, Ki = 0.01, Kd = 0.035;
    float previous_error_right = 0.0;
    float integral_right = 0.0;
    // Convert current RPM to motor speed percentage
    float current_speed_right = convertRPMToSpeed(current_rpm_right, desired_rpm_right);
    
    // Calculate error between desired and actual speed
    float error = max_speed - current_speed_right; // We want the speed to reach max_speed
    integral_right += error * dt;
    float derivative = (error - previous_error_right) / dt;

    // Calculate PID output
    float output = Kp * error + Ki * integral_right + Kd * derivative;
    previous_error_right = error;

    printf("Adjusted right motor speed: %.2f\n", output);

    return output;
}

void update_motor_speeds(float left_rpm, float right_rpm, float dt) {
    // Get PID outputs for both motors
    float left_pid_output = pid_control_left(left_rpm, dt);
    float right_pid_output = pid_control_right(right_rpm, dt);

    // Convert PID output to motor speed (normalized to 0-100% range)
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
        if (motor_control_enabled) {
            // Get current RPM from encoders (you need to implement these functions)
            float left_rpm = getLeftWheelRPM(100);  // Example: measure left wheel RPM over 100ms
            float right_rpm = getRightWheelRPM(100);  // Example: measure right wheel RPM over 100ms

            // Calculate the time difference (dt) between iterations
            absolute_time_t current_time = get_absolute_time();
            float dt = absolute_time_diff_us(last_time, current_time) / 1e6;  // in seconds
            last_time = current_time;

            // Adjust motor speeds based on PID controller output
            update_motor_speeds(left_rpm, right_rpm, dt);

            // Delay for a short time to avoid busy-waiting
            vTaskDelay(pdMS_TO_TICKS(100));  // Adjust the delay as necessary
        } else {
            
            continue;
            vTaskDelay(pdMS_TO_TICKS(1000));
            
        }
        
    }
}


void ultrasonic_wheel_speed_isr(uint gpio, uint32_t events) {
    if (gpio == ROTARY_PIN_L) {
        global_pulse_count_left++;
        global_pulse_count_for_turn_right++;
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
            xMessageBufferSend(objectDistanceMessageBuffer, &distance, sizeof(distance), portMAX_DELAY);
        }

    }
}


// Send trigger pulse to ultrasonic sensor
void send_trigger_pulse() {
    gpio_put(ULTRA_TRIG, 1);
    sleep_us(10); // Pulse duration
    gpio_put(ULTRA_TRIG, 0);
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

// Task to monitor distance and turn if necessary
void ultrasonic_task(void *pvParameters) {
    bool in_cooldown = false;

    while (true) {
        if (!in_cooldown && !stopped && !line_following_mode) {
            // Send a trigger pulse
            float distance = getCm(ULTRA_TRIG, ULTRA_ECHO);
            //xMessageBufferSend(objectDistanceMessageBuffer, &distance, sizeof(distance), portMAX_DELAY);

            // Check if an object is detected
            if (object_detected && !is_turning ) {
                printf("[Ultrasonic] Object detected at %.2f cm\n" , distance);
                
                motor_control_enabled = false;
                //turn_right_90(100);

                // Reset the object_detected flag and start cooldown
                object_detected = false;
                //is_turning = false;
                //printf("Turning flag is now false\n");
                in_cooldown = true;
                set_motor_direction(is_clockwise);
                set_left_motor_spd(70);
                set_right_motor_spd(70);
                //set_motor_spd(LEFT_MOTOR_PIN, 70);
                //set_motor_spd(RIGHT_MOTOR_PIN, 71); 
                //go_stop(90.0);
                //is_stopping = true;
                // Delay to prevent immediate re-triggering
                vTaskDelay(pdMS_TO_TICKS(500));  // Adjust the delay as needed
                printf("Cooldown period started\n");

            } else {
                if (is_moving) {
                    set_motor_direction(is_clockwise);
                    set_left_motor_spd(70);
                    set_right_motor_spd(70);
                    //set_motor_spd(LEFT_MOTOR_PIN, 70);
                    //set_motor_spd(RIGHT_MOTOR_PIN, 70); 
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
                last_printed_distance = distance;
            }
            
        }
    }
}

void turn_right() {
    set_left_motor_direction(is_clockwise);
    set_right_motor_direction(!is_clockwise);
    set_motor_spd(LEFT_MOTOR_PIN, 50);
    set_motor_spd(RIGHT_MOTOR_PIN, 50);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void turn_left() {
    set_left_motor_direction(!is_clockwise);
    set_right_motor_direction(is_clockwise);
    set_motor_spd(LEFT_MOTOR_PIN, 50);
    set_motor_spd(RIGHT_MOTOR_PIN, 50);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void ir_sensor_init() {
    gpio_init(IR_SENSOR_PIN_L);
    gpio_set_dir(IR_SENSOR_PIN_L, GPIO_IN);
    gpio_pull_down(IR_SENSOR_PIN_L);

    gpio_init(IR_SENSOR_PIN_R);
    gpio_set_dir(IR_SENSOR_PIN_R, GPIO_IN);
    gpio_pull_down(IR_SENSOR_PIN_R);

    gpio_init(line_follow_toggle_btn);
    gpio_set_dir(line_follow_toggle_btn, GPIO_IN);
    gpio_pull_down(line_follow_toggle_btn);
}

void line_following_task(void *pvParameters) {
    while (true) {
        if (gpio_get(line_follow_toggle_btn) == 0) {
            line_following_mode = !line_following_mode;
            printf("Line following mode: %s\n", line_following_mode ? "Enabled" : "Disabled");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (line_following_mode) {
            if (gpio_get(IR_SENSOR_PIN_R)==0) {
                printf("Left sensor: %d, Right sensor: %d\n, Moving forward", gpio_get(IR_SENSOR_PIN_L), gpio_get(IR_SENSOR_PIN_R));
                set_motor_direction(is_clockwise);
                set_motor_spd(LEFT_MOTOR_PIN, 50);
                set_motor_spd(RIGHT_MOTOR_PIN, 50);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (gpio_get(IR_SENSOR_PIN_R)==1) {
                printf("Left sensor: %d, Right sensor: %d\n, Turning left", gpio_get(IR_SENSOR_PIN_L), gpio_get(IR_SENSOR_PIN_R));
                turn_left();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (gpio_get(IR_SENSOR_PIN_L)==1 && gpio_get(IR_SENSOR_PIN_R)==1) {
                printf("Stopped\n");
                set_left_motor_spd(0);
                set_right_motor_spd(0);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Main function
int main() {
    stdio_init_all();
    init_motor_control();
    ultSonicPinInit();
    encoderPinInit();
    ir_sensor_init();

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
    //xTaskCreate(motor_control_task, "Motor Control Task", 512, NULL, 1, NULL);
    xTaskCreate(line_following_task, "Line Following Task", 512, NULL, 1, NULL);
    //xTaskCreate(pid_speed_balancer, "PID Speed Balancer Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}
