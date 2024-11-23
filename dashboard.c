#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define SERVER_IP "172.20.10.11"  // Set the server's IP address
#define TCP_PORT 4242
#define BUF_SIZE 2048
#define CHANGE_THRESHOLD 20

// Define the structure expected from the server (updated structure)
typedef struct {
    float forward_spd;
    float backward_spd;
    float turn_right_spd;
    float turn_left_spd;
    bool stop;
    bool auto_mode;
} DirectionCommands;



DirectionCommands prev_commands = {0.0f, 0.0f, 0.0f, 0.0f, false, false};

typedef struct {
    float forward_spd;
    float backward_spd;
    float turn_right_spd;
    float turn_left_spd;
    bool stop;
    bool auto_mode;
    float detected_distance;
    float distance_travelled;
    float left_wheel_rpm;
    float right_wheel_rpm;
} CarStatus;

CarStatus prev_car_status = {0.0f, 0.0f, 0.0f, 0.0f, false, false, 0.0f, 0.0f, 0.0f, 0.0f};

typedef struct TCP_CLIENT_T_ {
    struct tcp_pcb *client_pcb;
    bool connected;
    uint8_t buffer_recv[BUF_SIZE];
    CarStatus car_status;

} TCP_CLIENT_T;

static TCP_CLIENT_T *tcp_client_init(void) {
    TCP_CLIENT_T *state = calloc(1, sizeof(TCP_CLIENT_T));
    if (!state) {
        printf("Failed to allocate memory for TCP client state.\n");
        return NULL;
    }
    return state;
}

static err_t tcp_client_close(TCP_CLIENT_T *state) {
    if (state->client_pcb) {
        tcp_arg(state->client_pcb, NULL);
        tcp_close(state->client_pcb);
        state->client_pcb = NULL;
    }
    free(state);
    return ERR_OK;
}

static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;

    if (!p) {  // Server has closed the connection
        printf("Connection closed by the server.\n");
        tcp_client_close(state);
        return ERR_OK;
    }

    if (p->tot_len > 0) {
        pbuf_copy_partial(p, state->buffer_recv, p->tot_len, 0);
        state->buffer_recv[p->tot_len] = '\0';  // Null-terminate the received string

        if (p->tot_len == sizeof(CarStatus)) {
        // Copy the data into the buffer
        pbuf_copy_partial(p, &state->car_status, sizeof(CarStatus), 0);

/*         // Cast the buffer to the DirectionCommands struct and print its contents
        DirectionCommands *received_cmd = (DirectionCommands *)state->buffer_recv; */
        //printf("Receiving Direction Commands...\n");
        if (state->car_status.forward_spd > prev_car_status.forward_spd + CHANGE_THRESHOLD) {
            printf("[Accelerometer]  Forward Tilt: %.2f\n", state->car_status.forward_spd);
        }
        if (state->car_status.backward_spd > prev_car_status.backward_spd + CHANGE_THRESHOLD) {
            printf("[Accelerometer]  Backward Tilt: %.2f\n", state->car_status.backward_spd);
        }
        if (state->car_status.turn_right_spd > prev_car_status.turn_right_spd + CHANGE_THRESHOLD) {
            printf("[Accelerometer]  Turn Right Tilt: %.2f\n", state->car_status.turn_right_spd);
        }
        if (state->car_status.turn_left_spd > prev_car_status.turn_left_spd + CHANGE_THRESHOLD) {
            printf("[Accelerometer]  Turn Left Tilt: %.2f\n", state->car_status.turn_left_spd);
        }
        if (state->car_status.stop != prev_car_status.stop) {
            printf("[Accelerometer] Stop: %s\n", state->car_status.stop ? "True" : "False");
        }
        if (state->car_status.auto_mode != prev_car_status.auto_mode) {
            printf("[Accelerometer]  Auto Mode: %s\n", state->car_status.auto_mode ? "True" : "False");
        }
        if (state->car_status.detected_distance != prev_car_status.detected_distance) {
            printf("[ULTRASONIC]  Detected Object Distance: %.2f\n", state->car_status.detected_distance);
        }
        if (state->car_status.distance_travelled != prev_car_status.distance_travelled) {
            printf("[ENCODER]  Distance Travelled: %.2f\n", state->car_status.distance_travelled);
        }
        if (state->car_status.left_wheel_rpm != prev_car_status.left_wheel_rpm) {
            printf("[ENCODER]  Left Wheel RPM: %.2f\n", state->car_status.left_wheel_rpm);
        }
        if (state->car_status.right_wheel_rpm != prev_car_status.right_wheel_rpm) {
            printf("[ENCODER]  Right Wheel RPM: %.2f\n", state->car_status.right_wheel_rpm);
        }
        

        prev_car_status = state->car_status;  // Update the previous commands

        // Indicate that the data has been received
        tcp_recved(tpcb, p->tot_len);
        } else {
            printf("Received data size mismatch. Expected %zu bytes, got %u bytes.\n", sizeof(CarStatus), p->tot_len);
        }
    }

    pbuf_free(p);  // Free the received buffer
    return ERR_OK; // Continue receiving
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;

    if (err != ERR_OK) {
        printf("Connection error: %d\n", err);
        tcp_client_close(state);
        return err;
    }

    printf("Successfully connected to the server.\n");
    state->connected = true;
    tcp_recv(tpcb, tcp_client_recv); // Set the receive callback
    return ERR_OK;
}

static bool tcp_client_connect(TCP_CLIENT_T *state) {
    state->client_pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!state->client_pcb) {
        printf("Failed to create new TCP control block.\n");
        return false;
    }

    ip4_addr_t server_addr;
    if (!ip4addr_aton(SERVER_IP, &server_addr)) {
        printf("Invalid server IP address format.\n");
        return false;
    }

    err_t err = tcp_connect(state->client_pcb, &server_addr, TCP_PORT, tcp_client_connected);
    if (err != ERR_OK) {
        printf("Failed to initiate TCP connection: %d\n", err);
        tcp_client_close(state);
        return false;
    }

    tcp_arg(state->client_pcb, state);
    return true;
}

void run_tcp_client_test(void) {
    TCP_CLIENT_T *state = tcp_client_init();
    if (!state) {
        return;
    }

    if (!tcp_client_connect(state)) {
        tcp_client_close(state);
        return;
    }

    while (true) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(500));
    }
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("WiFi initialization failed\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("WiFi connection failed\n");
        return 1;
    }

    run_tcp_client_test();
    cyw43_arch_deinit();
    return 0;
}
