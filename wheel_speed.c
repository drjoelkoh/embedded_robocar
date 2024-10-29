#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include <stdio.h>

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

#define Kp 1.0
#define Ki 0.5
#define Kd 0.1
#define dt 0.5

bool is_clockwise = true;
float motor_speed = 100;
float pwm_clock = 0;
uint16_t wrap_value = 0;
MessageBufferHandle_t directionMessageBuffer;
MessageBufferHandle_t speedMessageBuffer; 

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
    wrap_value = (uint16_t)(pwm_clock / pwm_freq) - 1;
    pwm_set_clkdiv(slice_num1, 64); 
    pwm_set_wrap(slice_num1, wrap_value);
    pwm_set_clkdiv(slice_num2, 64); 
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
    gpio_put(DIR_PIN1, !clockwise); 
    gpio_put(DIR_PIN2, clockwise);
    gpio_put(DIR_PIN3, !clockwise); 
    gpio_put(DIR_PIN4, clockwise);
}

void set_motor_spd(int pin, float speed_percent) {
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;
    uint16_t duty_cycle = (uint16_t)((speed_percent / 100.0) * wrap_value); 
    pwm_set_gpio_level(pin, duty_cycle);
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
                is_clockwise = !is_clockwise;
                set_motor_direction(is_clockwise);
                printf("Direction toggled: %s\n", is_clockwise ? "Clockwise" : "Counterclockwise");
                printf(msg);
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
                printf("Speed set to LOW%%\n"); // Update log message
                printf(msg);
            } else if (msg == HISPD_BTN) {
                motor_speed = 100;
                set_motor_spd(PWM_PIN1, motor_speed);
                set_motor_spd(PWM_PIN2, motor_speed);
                printf("Speed set to HIGH%%\n");
                printf(msg);
            }
        }
    }
}

//implement ltr
double get_remote_control_speed() {
    return 100;
}

bool get_remote_control_direction() {
    //return left right forward backward
    return true;
}

double wheel_encoder_get_speed(int pin) {
    return 50;
}


// Main function
int main() {
    stdio_init_all();
    init_motor_control();

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

    // Set up interrupts
    gpio_set_irq_enabled_with_callback(DIR_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(LOWSPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled_with_callback(HISPD_BTN, GPIO_IRQ_EDGE_FALL, true, &button_isr);

    xTaskCreate(direction_task, "Direction Task", 512, NULL, 1, NULL);
    xTaskCreate(speed_task, "Speed Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}
