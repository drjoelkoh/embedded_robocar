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
// pid controller
#define Kp 1.0
#define Ki 0.5
#define Kd 0.1
#define dt 0.5

bool is_clockwise = true;
bool turning_right = false;
float motor_speed = 100;
float pwm_clock = 0;
uint16_t wrap_value = 0;
MessageBufferHandle_t directionMessageBuffer;
MessageBufferHandle_t speedMessageBuffer; 
MessageBufferHandle_t objectDistanceMessageBuffer;
MessageBufferHandle_t wheelSpeedMessageBuffer;

//ultrasonic and ir sensor stuff
#define ULTRA_TRIG 7
#define ULTRA_ECHO 6
#define ROTARY_PIN 16 // not needed now rmb to change pin number
int timeout = 26100;
volatile uint32_t global_pulse_count = 0;
absolute_time_t last_time;  // tracks previous point in time that the timing pulse was generated

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

bool debounce_button(uint gpio) {
    static absolute_time_t last_interrupt_time = {0};
    absolute_time_t current_time = get_absolute_time();
    int64_t time_diff = absolute_time_diff_us(last_interrupt_time, current_time);
    if (time_diff > DEBOUNCE_DELAY_MS * 1000) {
        last_interrupt_time = current_time;
        return true;
    }
    return false;
}

void set_motor_direction(bool clockwise) {
    gpio_put(DIR_PIN1, !clockwise); gpio_put(DIR_PIN2, clockwise);
    gpio_put(DIR_PIN3, !clockwise); gpio_put(DIR_PIN4, clockwise);
    printf("Direction set to %s\n", clockwise ? "Clockwise" : "Counterclockwise");
    
}

void set_motor_spd(int pin, float speed_percent) {
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;
    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value); 
    pwm_set_gpio_level(pin, duty_cycle);
    printf("Set speed of pin %d to %f\n", pin, speed_percent);
}

void turn_right(bool turn_right) {
    if (turn_right) {
        gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 0);
        gpio_put(DIR_PIN3, 0); gpio_put(DIR_PIN4, 0); 
        sleep_ms(1000);
        printf("Turning right\n");
        gpio_put(DIR_PIN1, 0); gpio_put(DIR_PIN2, 1); //clockwise
        gpio_put(DIR_PIN3, 1); gpio_put(DIR_PIN4, 0); //anticlockwise
        /* set_motor_spd(PWM_PIN1, 100);
        set_motor_spd(PWM_PIN2, 50); */
    }
    else {
        set_motor_direction(is_clockwise);
    }

    
}

// Interrupt Service Routine for button presses
void button_isr(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (events & GPIO_IRQ_EDGE_FALL) {
        if (!debounce_button(gpio)) return; // Debounce check

        uint32_t msg;
        if (gpio == DIR_BTN) {
            msg = DIR_BTN;
            xMessageBufferSendFromISR(directionMessageBuffer, &msg, sizeof(msg), &xHigherPriorityTaskWoken);
        } else if (gpio == LOWSPD_BTN) {
            msg = LOWSPD_BTN;
            xMessageBufferSendFromISR(speedMessageBuffer, &msg, sizeof(msg), &xHigherPriorityTaskWoken);
        } else if (gpio == HISPD_BTN) {
            msg = HISPD_BTN;
            xMessageBufferSendFromISR(speedMessageBuffer, &msg, sizeof(msg), &xHigherPriorityTaskWoken);
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task to handle motor direction based on button input
void direction_task(void *pvParameters) {
    while (true) {
        uint32_t msg;
        if (xMessageBufferReceive(directionMessageBuffer, &msg, sizeof(msg), portMAX_DELAY)) {
            if (msg == DIR_BTN) {
                /* is_clockwise = !is_clockwise;
                set_motor_direction(is_clockwise);
                printf("Direction toggled: %s\n", is_clockwise ? "Clockwise" : "Counterclockwise"); */
                turn_right(!turning_right);
                turning_right = !turning_right;
            }
            
        }
    }
}

// Task to handle motor speed based on button input
void speed_task(void *pvParameters) {
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
}

//implement ltr
float get_remote_control_speed() {
    return 100;
}

bool get_remote_control_direction() {
    //return left right forward backward
    return true;
}

double wheel_encoder_get_speed(int pin) {
    return 50;
}

/* Initialise the Photo Interrupt sensor input pin */
void encoderPinInit() {
    gpio_init(ROTARY_PIN);
    gpio_set_dir(ROTARY_PIN, GPIO_IN);
    gpio_pull_down(ROTARY_PIN); // (HIGH=1,LOW=0)
    
}
/* Increase pulse count when a rising edge of disc pulse is detected */
void sensor_pulse_interrupt_handler(uint gpio, uint32_t events) {
    global_pulse_count++;
}

/* Calculates number of pulses divided by timeframe to get speed at point in time*/
float getRevsPerMin(uint32_t timeframe_pulse_counts, float duration_sec) {
    return (timeframe_pulse_counts / duration_sec) * 60.0f;
}

/* Uses the photo interrupter sensor to measure the rotational speed of the encoder
(Final version  will require wheel circumference specs to gauge distance and speed of car)
ARGS: sample+time_ms: duration which the wheelspeed is measured over (ms)
1. Wait for the regularly timed pulse edge trigger to start the measurement
2. In the period of one pulse, count the number of times that the disc blocks the sensor (1-0 trigger)
3. Divide the disc_pulse_count value by the duration of one time_pulse 
4. 
5. [To be implemented later] Multiply rotational speed by car wheel circumference to get actual distance and speed*/
float getWheelSpeed(float sample_time_ms) {
    // measure the RPM at 
    absolute_time_t current_time;
    float duration_seconds, revs_per_min;

    sleep_ms(sample_time_ms); // 
    current_time = get_absolute_time();
    duration_seconds = absolute_time_diff_us(last_time, current_time) / 1e6;  //time difference in microseconds then convert into seconds
    revs_per_min = getRevsPerMin(global_pulse_count, duration_seconds);
    printf("[Encoder] Pulses: %ld\n", (long)global_pulse_count);

    global_pulse_count = 0; // reset pulse count
    last_time = current_time;   // set 'new' time to reference previous time from
    return revs_per_min;
}
void wheel_speed_task(void *pvParameters) {
    const TickType_t sampleTimeTicks = pdMS_TO_TICKS(1000);
    while (true) {
        float speed = getWheelSpeed(100);  
        xMessageBufferSend(wheelSpeedMessageBuffer, &speed, sizeof(speed), portMAX_DELAY);
        vTaskDelay(sampleTimeTicks); // Delay for the sampling period
    }
}
// Task to receive and print wheel speed
void printWheelSpeedTask(void *pvParameters) {
    float speed;
    while (true) {
        if (xMessageBufferReceive(wheelSpeedMessageBuffer, &speed, sizeof(speed), portMAX_DELAY) > 0) {
            printf("[Encoder] Wheel speed: %.2f RPM\n", speed);
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

float desired_distance = 10.0; // desired distance in cm
float prev_error = 0.0, integral = 0.0;

float pid_controller(float current_distance) {
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

void ultrasonic_task(void *pvParameters) {
    bool in_cooldown = false;
    while (true) {
        if (!in_cooldown) {
            float distance = getCm(ULTRA_TRIG, ULTRA_ECHO);
            xMessageBufferSend(objectDistanceMessageBuffer, &distance, sizeof(distance), portMAX_DELAY);

            if (distance <= 10.0 && distance > 1.0) { // If object is within 10 cm, trigger PID-based turn
                printf("Object detected at %.2f cm\n", distance);
                float turn_speed = pid_controller(distance); // Use PID controller to get turn speed
                set_motor_spd(PWM_PIN1, turn_speed);         // Adjust speed of each wheel to turn
                set_motor_spd(PWM_PIN2, turn_speed / 1.5);     // Adjust one wheel slower to turn
                turn_right(turning_right);                            // Set turning right action
                turning_right = !turning_right;                 // Toggle turning direction */
                in_cooldown = true;                          // Activate cooldown to stop further scanning
            } else if (distance == 0.0 || distance > 10.0) {
                set_motor_direction(is_clockwise);           // Maintain forward motion if no object detected
                set_motor_spd(PWM_PIN1, 100);
                set_motor_spd(PWM_PIN2, 100);
            }
        } else {
            // Cooldown period after detecting an obstacle, e.g., 1 second
            vTaskDelay(pdMS_TO_TICKS(1000));                 // Delay for 1 second to prevent re-triggering
            in_cooldown = false;                             // Reset cooldown to resume scanning
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Main task delay for periodic distance checking
    }
}


void print_dist_task(void *pvParameters) {
    float distance;
    while (true) {
        if (xMessageBufferReceive(objectDistanceMessageBuffer, &distance, sizeof(distance), portMAX_DELAY) > 0) {
            printf("[Ultrasonic] Distance from object: %.2f cm\n", distance);
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

    wheelSpeedMessageBuffer = xMessageBufferCreate(64);
    if (wheelSpeedMessageBuffer == NULL) {
        printf("Wheel speed message buffer creation failed!\n");
        return 1;
    }

    // Set up interrupts
    /* gpio_set_irq_enabled_with_callback(DIR_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(LOWSPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(HISPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr); */
    
    gpio_set_irq_enabled_with_callback(ROTARY_PIN, GPIO_IRQ_EDGE_RISE, true, &sensor_pulse_interrupt_handler);

    xTaskCreate(direction_task, "Direction Task", 512, NULL, 1, NULL);
    xTaskCreate(speed_task, "Speed Task", 512, NULL, 1, NULL);
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 512, NULL, 1, NULL);
    xTaskCreate(print_dist_task, "Print Distance Task", 512, NULL, 1, NULL);
    xTaskCreate(wheel_speed_task, "Wheel Speed Task", 512, NULL, 1, NULL);
    xTaskCreate(printWheelSpeedTask, "Print Wheel Speed Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();

/*     uint64_t obj_distance = 0;
    while (true) {
        obj_distance = getCm(ULTRA_TRIG, ULTRA_ECHO);
        printf("[Ultrasonic] Distance from object %.2ld cm\n", (long)obj_distance);
    } */

    return 0;
}
