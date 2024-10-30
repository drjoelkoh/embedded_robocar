/*
Tasks:
1. Measure distance between ultrasonic sensor, HC-SR04P, and object
2. Capture Pulse Width data from wheel encoder during rotation
*/
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

#define ULTRA_TRIG 1
#define ULTRA_ECHO 0
#define ROTARY_PIN 4 // tbh idk what pin, might be diff

/* Global Vars for Ultrasonic Sensor */
volatile uint32_t echo_start_time = 0;
volatile uint32_t echo_end_time = 0;
volatile uint32_t echo_time_diff = 0;

/* Global Vars for Wheel Encoder */
int timeout = 26100;
volatile uint32_t global_pulse_count = 0;
absolute_time_t last_time;  // tracks previous point in time that the timing pulse was generated

/* Initialise the Ultrasonic sensor trigger and echo pins*/
void ultSonicPinInit() {
    gpio_init(ULTRA_TRIG);
    gpio_init(ULTRA_ECHO);
    gpio_set_dir(ULTRA_TRIG, GPIO_OUT); // Pin sends the pulse of sound wave out
    gpio_set_dir(ULTRA_ECHO, GPIO_IN);  // Pin receives the data of incoming reflected sound wave
}

void send_pulse(uint trigger_pin) {
    gpio_put(trigger_pin, 1);
    sleep_us(10);
    gpio_put(trigger_pin, 0);
}

/* Return time difference between when echo_pin goes from 1 to 0*/
uint32_t echo_pulse(uint trigger_pin, uint echo_pin) {
    gpio_put(trigger_pin, 1);   //sends out a HIGH signal from trigger_pin
    sleep_us(10);
    gpio_put(trigger_pin, 0);   // waits a duration of 10us before going back to LOW

    uint64_t width = 0;
    
    while (gpio_get(echo_pin) == 0) tight_loop_contents(); // wait until echo_pin == 1 (receive echo)
    uint32_t start_time = time_us_32();    // get time when pin goes HIGH
    while (gpio_get(echo_pin) == 1)// wait until echo_pin goes back to 0 implement interrupt for timing instead of blocking
    {
        width++;
        sleep_us(1);    // wait 1 microsecond
        if (width > timeout) return 0;
    }
    uint32_t end_time = time_us_32();  // get the time when pin goes LOW
    uint32_t time_diff = end_time - start_time;
    return time_diff;
}

/* ISR for Echo Pin receiving return pulse.
    Sets start time when pin detected HIGH
    Sets end time when pin goes LOW */
void echoPinReceiveInterrupt() {
    switch (gpio_get_out_level(ULTRA_ECHO)) {// check signal HIGH or LOW
        case true:  //HIGH
            echo_end_time =  0;   // clears previous end timing
            echo_start_time = time_us_32();   // sets new start timing
            printf("ECHO RECEIVED Start time: %.2u\n", echo_start_time);
            break;
        case false:
            echo_end_time = time_us_32();
            echo_time_diff = echo_end_time - echo_start_time;   // calculate the duration of the echo
            printf("End time: %.2u\n", echo_end_time);
            //sleep_ms(10);
            break;
    }
}

float getCm(uint trigPin, uint echoPin) {
    uint32_t pulseLength = echo_pulse(trigPin, echoPin);
    float distance = (pulseLength / 2.0 ) * 0.0343;    // speed of sound (echo) is 34300 cm/s
    return distance;
}

float getCm2(uint trigPin) {
    send_pulse(trigPin);
    uint32_t pulseLength = echo_time_diff;
    float distance = ( pulseLength / 2.0 ) * 0.0343;
    return distance;
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
    printf("Pulses: %ld\n", (long)global_pulse_count);

    global_pulse_count = 0; // reset pulse count
    last_time = current_time;   // set 'new' time to reference previous time from
    return revs_per_min;
}

int main() {
    stdio_init_all();
    ultSonicPinInit();
    encoderPinInit();

    /*Just uncomment these parts to show the ultrasonic sensor*/
    // printf("Started measuring object distance");
    float obj_distance = 0.0;
    float obj_distance2 = 0.0;
    // while(true) {
    //     obj_distance = getCm(ULTRA_TRIG, ULTRA_ECHO);
    //     printf("[Ultrasonic] Distance from object %.2ld cm/n", (long)obj_distance);
    // }
    gpio_set_irq_enabled_with_callback(ULTRA_ECHO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echoPinReceiveInterrupt);
    //gpio_set_irq_enabled_with_callback(ROTARY_PIN, GPIO_IRQ_EDGE_RISE, true, &sensor_pulse_interrupt_handler);  // interrupts for counting wheel encoder pulses
    printf("Measuring rotation speed of wheel");
    float wheel_speed;
    while (true) {
        obj_distance = getCm(ULTRA_TRIG, ULTRA_ECHO);
        obj_distance2 = getCm2(ULTRA_TRIG);
        printf("[Ultrasonic] Distance from object %.2f cm\n", obj_distance);
        printf("[Ultrasonic Int] Distance from obj %.2f cm\n", obj_distance2);
        wheel_speed = getWheelSpeed(1000);
        printf("[Wheel Encoder] The wheel is rotating at %.2ld m/s\n", (long)wheel_speed);
    }
}