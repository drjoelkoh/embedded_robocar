#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include <math.h>

// I2C setup and accelerometer/magnetometer constants
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

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

#define TOGGLE_BTN 20

#define TCP_PORT 4242
#define SERVER_IP "172.20.10.11" // Update with server IP 172.20.10.11
#define CHANGES_THRESHOLD 20

static struct tcp_pcb *client_pcb;
static bool connected = false;

int raw_xa, raw_ya, raw_za;
int raw_xm, raw_ym, raw_zm;

// Kalman filter struct
typedef struct {
    float q;
    float r;
    float x;
    float p;
    float k;
} KalmanFilter;

typedef struct {
    float forward_spd;
    float backward_spd;
    float turn_right_spd;
    float turn_left_spd;
    bool stop;
    bool auto_mode;
} DirectionCommands;

DirectionCommands previous_commands = {0.0f, 0.0f, 0.0f, 0.0f, false, false};

KalmanFilter kalman_xa = {0.05, 0.05, 0, 1, 0}; // Lower process noise for smoother readings
KalmanFilter kalman_ya = {0.05, 0.05, 0, 1, 0};
KalmanFilter kalman_za = {0.05, 0.05, 0, 1, 0};

KalmanFilter kalman_xm = {0.05, 0.05, 0, 1, 0};
KalmanFilter kalman_ym = {0.05, 0.05, 0, 1, 0};
KalmanFilter kalman_zm = {0.05, 0.05, 0, 1, 0};

void init_toggle_btn() {
    gpio_init(TOGGLE_BTN);
    gpio_set_dir(TOGGLE_BTN, GPIO_IN);
    gpio_pull_up(TOGGLE_BTN);
}

void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, address, buffer, 2, false);
}

void i2c_read_bytes(uint8_t address, uint8_t reg, uint8_t* buffer, int length) {
    i2c_write_blocking(I2C_PORT, address, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, address, buffer, length, false);
}

void setup_accelerometer() {
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
    kf->p += kf->q;
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1 - kf->k);
    return kf->x;
}

void calculate_tilt(float xa, float ya, float za, float xm, float ym, float zm, float *pitch, float *roll, float *yaw) {
    *pitch = atan2(xa, sqrt(ya * ya + za * za)) * 180.0 / M_PI;
    *roll = atan2(ya, sqrt(xa * xa + za * za)) * 180.0 / M_PI;
    *yaw = atan2(-ym, xm) * 180.0 / M_PI;
    if (yaw < 0) {
        yaw += 360; // Ensure yaw is within 0-360 degrees
    }
}

void set_previous_commands(DirectionCommands *prev_commands, DirectionCommands *dir_commands) {
    prev_commands->forward_spd = dir_commands->forward_spd;
    prev_commands->backward_spd = dir_commands->backward_spd;
    prev_commands->turn_right_spd = dir_commands->turn_right_spd;
    prev_commands->turn_left_spd = dir_commands->turn_left_spd;
    prev_commands->stop = dir_commands->stop;
    prev_commands->auto_mode = dir_commands->auto_mode;
}

bool compare_previous(DirectionCommands *prev_commands, DirectionCommands *dir_commands) {
    if (fabs(dir_commands->forward_spd - prev_commands->forward_spd) > CHANGES_THRESHOLD) {
        return true;
    }
    if (fabs(dir_commands->backward_spd - prev_commands->backward_spd) > CHANGES_THRESHOLD) {
        return true;
    }
    if (fabs(dir_commands->turn_right_spd - prev_commands->turn_right_spd) > CHANGES_THRESHOLD) {
        return true;
    }
    if (fabs(dir_commands->turn_left_spd - prev_commands->turn_left_spd) > CHANGES_THRESHOLD) {
        return true;
    }
    if (dir_commands->stop != prev_commands->stop) {
        return true;
    }
    if (dir_commands->auto_mode != prev_commands->auto_mode) {
        return true;
    }
    return false;
}

void read_sensor_data() {
    uint8_t accel_data_x[2], accel_data_y[2], accel_data_z[2];
    uint8_t mag_data_x[2], mag_data_y[2], mag_data_z[2];

    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_X_L_A | 0x80, accel_data_x, 2);
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_Y_L_A | 0x80, accel_data_y, 2);
    i2c_read_bytes(ACC_ADDRESS, ACC_OUT_Z_L_A | 0x80, accel_data_z, 2);
    raw_xa = (int16_t)(accel_data_x[0] | (accel_data_x[1] << 8));
    raw_ya = (int16_t)(accel_data_y[0] | (accel_data_y[1] << 8));
    raw_za = (int16_t)(accel_data_z[0] | (accel_data_z[1] << 8));

    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_X_H_M, mag_data_x, 2);
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_Y_H_M, mag_data_y, 2);
    i2c_read_bytes(MAG_ADDRESS, MAG_OUT_Z_H_M, mag_data_z, 2);
    raw_xm = (int16_t)((uint16_t)mag_data_x[1] | ((uint16_t)mag_data_x[0] << 8));
    raw_ym = (int16_t)((uint16_t)mag_data_y[1] | ((uint16_t)mag_data_y[0] << 8));
    raw_zm = (int16_t)((uint16_t)mag_data_z[1] | ((uint16_t)mag_data_z[0] << 8));
}

static err_t tcp_client_close(void) {
    if (client_pcb) {
        tcp_arg(client_pcb, NULL);
        tcp_close(client_pcb);
        client_pcb = NULL;
    }
    return ERR_OK;
}

static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        printf("Server disconnected\n");
        return tcp_client_close();
    }
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Failed to connect to server\n");
        return tcp_client_close();
    }
    printf("Connected to server\n");
    tcp_recv(tpcb, tcp_client_recv);
    connected = true;
    return ERR_OK;
}

static void run_tcp_client(void) {
    client_pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!client_pcb) {
        printf("Failed to create pcb\n");
        return;
    }
    ip4_addr_t server_ip;
    ip4addr_aton(SERVER_IP, &server_ip);
    err_t err = tcp_connect(client_pcb, &server_ip, TCP_PORT, tcp_client_connected);
    if (err != ERR_OK) {
        printf("Failed to connect to server: %d\n", err);
        tcp_client_close();
    }
}

static void send_pitch_roll_yaw(float pitch, float roll, float yaw) {
    int max_pitch = 80; // Define the maximum pitch angle for full speed
    int max_roll = 80;  // Define the maximum roll angle for full turn speed

    // Calculate forward/backward speed as a percentage
    int speed = (int)((fabs(pitch) / max_pitch) * 100); // Speed percentage
    DirectionCommands dir_commands = {0.0f, 0.0f, 0.0f, 0.0f, false, false};
    
    if (connected && client_pcb) {
        char message[100];
        snprintf(message, sizeof(message), "Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", pitch, roll, yaw);
        if (speed > 100) speed = 100; // Cap at 100%

        // Calculate turn intensity as a percentage
        int turn_speed = (int)((fabs(roll) / max_roll) * 100); // Turn percentage
        if (turn_speed > 100) turn_speed = 100; // Cap at 100%

        // Determine the direction of movement
        if (pitch > 10) {
            snprintf(message, sizeof(message), "Command: Backward at %d%% speed\n", speed);
            dir_commands.backward_spd = speed;
        } else if (pitch < -10) {
            snprintf(message, sizeof(message), "Command: Forward at %d%% speed\n", speed);
            dir_commands.forward_spd = speed;
        } else {
            snprintf(message, sizeof(message), "Command: Stop\n");
            dir_commands.stop = !dir_commands.stop;
        }

        // Determine the turning direction
        if (roll > 10) {
            snprintf(message, sizeof(message), "Command: Turn Right at %d%% speed\n", turn_speed);
            dir_commands.turn_right_spd = turn_speed;
        } else if (roll < -10) {
            snprintf(message, sizeof(message), "Command: Turn Left at %d%% speed\n", turn_speed);
            dir_commands.turn_left_spd = turn_speed;
        }

        if (gpio_get(TOGGLE_BTN)==0) {
            dir_commands.auto_mode = !dir_commands.auto_mode;
            printf("Auto mode: %s\n", dir_commands.auto_mode ? "Enabled" : "Disabled");
            sleep_ms(1000);
        }
        
        if (compare_previous(&previous_commands, &dir_commands)) {
            //err_t err = tcp_write(client_pcb, message, strlen(message), TCP_WRITE_FLAG_COPY);
            err_t err = tcp_write(client_pcb, &dir_commands, sizeof(dir_commands), TCP_WRITE_FLAG_COPY);
            set_previous_commands(&previous_commands, &dir_commands);
            if (err == ERR_OK) {
                tcp_output(client_pcb);
                printf("Sent: %s\n", message);
            } else if (err == ERR_MEM) {
                printf("TCP buffer full, retrying\n");
                sleep_ms(10); // Short delay to prevent overloading
            }
        }
        /* else {
            printf("Failed to send message\n");
        } */
    } else {
        printf("Not connected to server\n");
    }
}



int main() {
    stdio_init_all();
    init_toggle_btn();
    setup_accelerometer();
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Failed to connect to WiFi\n");
        return 1;
    }
    printf("WiFi connected\n");
    run_tcp_client();
    while (true) {
        cyw43_arch_poll();
        read_sensor_data();
        float pitch, roll, yaw;
        float filtered_xa = kalman_filter(&kalman_xa, (float)raw_xa);
        float filtered_ya = kalman_filter(&kalman_ya, (float)raw_ya);
        float filtered_za = kalman_filter(&kalman_za, (float)raw_za);
        
        float filtered_xm = kalman_filter(&kalman_xm, (float)raw_xm);
        float filtered_ym = kalman_filter(&kalman_ym, (float)raw_ym);
        float filtered_zm = kalman_filter(&kalman_zm, (float)raw_zm);
        calculate_tilt(filtered_xa, filtered_ya, filtered_za, filtered_xm, filtered_ym, filtered_zm, &pitch, &roll, &yaw);
        send_pitch_roll_yaw(pitch, roll, yaw);
        sleep_ms(500);
    }
    cyw43_arch_deinit();
    return 0;
}
