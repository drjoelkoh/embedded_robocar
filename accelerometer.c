#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

// Define I2C port and pins
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

// Device addresses and registers
#define ACC_ADDRESS 0x19
#define ACC_CTRL_REG1_A 0x20
#define ACC_OUT_X_L_A 0x28
#define ACC_OUT_Y_L_A 0x2A
#define ACC_OUT_Z_L_A 0x2C

#define MAG_ADDRESS 0x1E
#define MAG_MR_REG_M 0x02
#define MAG_OUT_X_H_M 0x03
#define MAG_OUT_Y_H_M 0x05
#define MAG_OUT_Z_H_M 0x07

int raw_xa, raw_ya, raw_za;
int raw_xm, raw_ym, raw_zm;

// Kalman filter variables for accelerometer and magnetometer
typedef struct {
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Value
    float p; // Estimation error covariance
    float k; // Kalman gain
} KalmanFilter;

KalmanFilter kalman_xa = {0.1, 0.1, 0, 1, 0};
KalmanFilter kalman_ya = {0.1, 0.1, 0, 1, 0};
KalmanFilter kalman_za = {0.1, 0.1, 0, 1, 0};

KalmanFilter kalman_xm = {0.1, 0.1, 0, 1, 0};
KalmanFilter kalman_ym = {0.1, 0.1, 0, 1, 0};
KalmanFilter kalman_zm = {0.1, 0.1, 0, 1, 0};

void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, address, buffer, 2, false);
}

void i2c_read_bytes(uint8_t address, uint8_t reg, uint8_t* buffer, int length) {
    i2c_write_blocking(I2C_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, address, buffer, length, false);
}

void setup() {
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    i2c_write_byte(ACC_ADDRESS, ACC_CTRL_REG1_A, 0x57);
    i2c_write_byte(MAG_ADDRESS, MAG_MR_REG_M, 0x00);
}

float kalman_filter(KalmanFilter *kf, float measurement) {
    // Prediction update
    kf->p += kf->q;

    // Measurement update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1 - kf->k);

    return kf->x;
}

void calculate_tilt(float xa, float ya, float za) {
    // Calculate pitch and roll in degrees
    float pitch = atan2(xa, sqrt(ya * ya + za * za)) * 180.0 / M_PI;
    float roll = atan2(ya, sqrt(xa * xa + za * za)) * 180.0 / M_PI;
    float yaw = atan2(za, sqrt(xa * xa + ya * ya)) * 180.0 / M_PI;

    // Output easy-to-understand descriptions
    printf("Tilt Orientation:\n");

    // Forward/Backward tilt (Pitch)
    if (pitch > 5) {
        printf("Tilt: Forward (%.2f degrees)\n", pitch);
    } else if (pitch < -5) {
        printf("Tilt: Backward (%.2f degrees)\n", -pitch);
    } else {
        printf("Tilt: Level\n");
    }

    // Left/Right tilt (Roll)
    if (roll > 5) {
        printf("Tilt: Right (%.2f degrees)\n", roll);
    } else if (roll < -5) {
        printf("Tilt: Left (%.2f degrees)\n", -roll);
    } else {
        printf("Tilt: Centered\n");
    }

    printf("Rotation Angle: %.2f degrees\n", yaw);
}

void loop() {
    uint8_t accel_data_x[2], accel_data_y[2], accel_data_z[2];
    uint8_t mag_data_x[2], mag_data_y[2], mag_data_z[2];

    // Read accelerometer data
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_X_L_A | 0x80, accel_data_x, 2);
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_Y_L_A | 0x80, accel_data_y, 2);
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_Z_L_A | 0x80, accel_data_z, 2);
    raw_xa = (int16_t)(accel_data_x[0] | (accel_data_x[1] << 8));
    raw_ya = (int16_t)(accel_data_y[0] | (accel_data_y[1] << 8));
    raw_za = (int16_t)(accel_data_z[0] | (accel_data_z[1] << 8));

    // Read magnetometer data
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_X_H_M, mag_data_x, 2);
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_Y_H_M, mag_data_y, 2);
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_Z_H_M, mag_data_z, 2);
    raw_xm = (int16_t)((uint16_t)mag_data_x[1] | ((uint16_t)mag_data_x[0] << 8));
    raw_ym = (int16_t)((uint16_t)mag_data_y[1] | ((uint16_t)mag_data_y[0] << 8));
    raw_zm = (int16_t)((uint16_t)mag_data_z[1] | ((uint16_t)mag_data_z[0] << 8));

    // Apply Kalman filter
    float filtered_xa = kalman_filter(&kalman_xa, (float)raw_xa);
    float filtered_ya = kalman_filter(&kalman_ya, (float)raw_ya);
    float filtered_za = kalman_filter(&kalman_za, (float)raw_za);

    float filtered_xm = kalman_filter(&kalman_xm, (float)raw_xm);
    float filtered_ym = kalman_filter(&kalman_ym, (float)raw_ym);
    float filtered_zm = kalman_filter(&kalman_zm, (float)raw_zm);


    calculate_tilt(filtered_xa, filtered_ya, filtered_za);

    // Output filtered values
    //printf("Filtered Accelerometer: X=%.2f Y=%.2f Z=%.2f\n", filtered_xa, filtered_ya, filtered_za);
    //printf("Filtered Magnetometer: X=%.2f Y=%.2f Z=%.2f\n", filtered_xm, filtered_ym, filtered_zm);

    // Output raw values for reference
    //printf("Accelerometer: X=%d Y=%d Z=%d\t", raw_xa, raw_ya, raw_za);
    //printf("Magnetometer: X=%d Y=%d Z=%d\n", raw_xm, raw_ym, raw_zm);

    sleep_ms(1000); // Delay of 1000ms
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
